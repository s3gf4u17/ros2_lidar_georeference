# === MAKEFILE LOGGER ===

COLOR_succ := \033[0;32m
COLOR_warn := \033[0;33m
COLOR_info := \033[0;34m
COLOR_erro := \033[0;31m
COLOR_rest := \033[0m

define mk_log
	printf "%b[%s] %s%b\n" "$(COLOR_$(1))" "$(shell echo $(1) | tr a-z A-Z)" "$(2)" "$(COLOR_rest)"
endef

# === PACKAGE BUILDER === EXECUTE ON LOCAL PC ===

REQUIRED_PKGS := \
	fixposition_driver_lib \
	fixposition_driver_msgs \
	fixposition_driver_ros2 \
	fpsdk_apps \
	fpsdk_common \
	fpsdk_ros2 \
	ros2_lidar_georeference \
	rtcm_msgs \
	velodyne \
	velodyne_driver \
	velodyne_laserscan \
	velodyne_msgs \
	velodyne_pointcloud

define mk_check_pkg
	@for pkg in $(REQUIRED_PKGS); do \
		if [ ! -d install/$$pkg ]; then \
			$(call mk_log,erro,package missing $$pkg); \
			exit 4; \
		fi; \
	done
endef

build:
	@$(call mk_log,info,removing old build files)
	@rm -rf install/ upload.zip \
		|| {$(call mk_log,erro,failed to remove old build files); exit 1;}
	@$(call mk_log,info,creating docker builder image. this may take a moment)
	@docker build --quiet --platform linux/arm64 --tag ros2_lidar_georeference:builder . \
		|| {$(call mk_log,erro,failed to create docker builder image); exit 2;}
	@$(call mk_log,info,extracting install/ folder from the builder image)
	@container_id=$$(docker create ros2_lidar_georeference:builder) \
		&& docker cp $$container_id:/ws/install/ install/ \
		&& docker rm $$container_id \
		|| {$(call mk_log,erro,failed to copy install/ files); exit 3;}
	@$(call mk_check_pkg)
	@$(call mk_log,info,all required packages found. zipping install/)
	@zip -rq upload.zip install/ makefile \
		&& $(call mk_log,succ,build success) \
		|| $(call mk_log,erro,error creating a zip package)

# === PACKAGE UPLOAD === EXECUTE ON LOCAL PC ===

ROVER_USER := pi
ROVER_IP := 10.0.0.1
UPLOAD_DIRECTORY := /home/pi/

upload:
	@$(call mk_log,info,uploading deployment zip package to leo rover)
	@scp upload.zip $(ROVER_USER)@$(ROVER_IP):$(UPLOAD_DIRECTORY) \
		&& $(call mk_log,succ,zip package uploaded) \
		|| $(call mk_log,erro,error uploading a zip package)

# === NGINX INSTALL == EXECUTE ON LEO ROVER ===

NGINX_SITES_ENABLED := /etc/nginx/sites-enabled
SITE_NAME := ros2_lidar_georeference
SITE_PORT := 8888
PKG_ROOT := $(shell ros2 pkg prefix ros2_lidar_georeference)
WEB_ROOT := /var/www/$(SITE_NAME)
CONF_FILE := $(NGINX_SITES_ENABLED)/$(SITE_NAME)
DOWNLOAD_DIR := $(WEB_ROOT)/downloads
NGINX := nginx

nginx:
	@$(call mk_log,info,creating nginx files)
	@sudo mkdir -p $(DOWNLOAD_DIR) \
		|| {$(call mk_log,erro,failed to create folder in /var/www/); exit 4;}
	@sudo cp -r $(PKG_ROOT)/share/ros2_lidar_georeference/web/* $(WEB_ROOT) \
		|| {$(call mk_log,erro,failed to copy web files); exit 5;}
	@$(call mk_log,info,configuring webshare group)
	@sudo groupadd -f webshare \
		&& sudo usermod -a -G webshare pi \
		&& sudo usermod -a -G webshare www-data \
		&& sudo chown -R www-data:webshare $(WEB_ROOT) \
		&& sudo chmod -R 775 $(DOWNLOAD_DIR) \
		&& sudo chmod g+s $(DOWNLOAD_DIR) \
		|| {$(call mk_log,erro,failed to configure webshare group); exit 6;}
	@$(call mk_log,info,configuring nginx)
	@printf "%s\n" \
		"server {" \
		"    listen $(SITE_PORT);" \
		"    listen [::]:$(SITE_PORT);" \
		"    server_name _;" \
		"    root $(WEB_ROOT);" \
		"    index index.html;" \
		"    location = / {" \
		"        try_files /index.html =404;" \
		"    }" \
		"    location ^~ /css/ {" \
		"        try_files \$$uri \$$uri/ =404;" \
		"    }" \
		"    location ^~ /js/ {" \
		"        try_files \$$uri \$$uri/ =404;" \
		"    }" \
		"    location ^~ /downloads/ {" \
		"        try_files \$$uri =404;" \
		"        add_header Content-Disposition 'attachment';" \
		"    }" \
		"    location / {" \
		"        return 403;" \
		"    }" \
		"}" \
		| sudo tee $(CONF_FILE) > /dev/null \
		&& sudo $(NGINX) -t \
		&& sudo $(NGINX) -s reload \
		|| {$(call mk_log,erro,failed to configure nginx); exit 7;}
	@$(call mk_log,succ,web ui available)

# === PACKAGES CONFIGURE == EXECUTE ON LEO ROVER ===

VELODYNE_IP := 10.0.0.80
FIXPOSITION_IP := 10.0.0.137

configure:
	@$(call mk_log,info,setting velodyne ip)
	sed -i 's/192.168.1.201/$(VELODYNE_IP)/g' $(PKG_ROOT)/../velodyne_driver/share/velodyne_driver/config/VLP16-velodyne_driver_node-params.yaml
	@$(call mk_log,info,setting fixposition ip)
	sed -i 's/10.0.2.1/$(FIXPOSITION_IP)/g' $(PKG_ROOT)/../fixposition_driver_ros2/share/fixposition_driver_ros2/launch/config.yaml

# === PROJECT RUN === EXECUTE ON LEO ROVER ===

run:
	@screen -dmS ros2_launch bash -c ' \
		source /opt/ros/jazzy/setup.bash && \
		source install/setup.bash && \
		ros2 launch ros2_lidar_georeference file_manager_with_rosbridge.launch.py'

# === CLEAN ENVIRONMENT === EXECUTE ON EITHER ===

clean:
	@if grep -q "Raspberry Pi 4" /proc/device-tree/model 2>/dev/null; then \
		$(call mk_log,warn,cleaning rover environment); \
		if [ -f $(CONF_FILE) ]; then \
			$(call mk_log,info,removing nginx config); \
			sudo rm -f $(CONF_FILE); \
		else \
			$(call mk_log,info,nginx config not found, skipping); \
		fi; \
		if [ -d $(WEB_ROOT) ]; then \
			$(call mk_log,info,removing web root $(WEB_ROOT)); \
			sudo rm -rf $(WEB_ROOT); \
		else \
			$(call mk_log,info,web root not found, skipping); \
		fi; \
		$(call mk_log,info,removing users from webshare group); \
		sudo gpasswd -d pi webshare 2>/dev/null || true; \
		sudo gpasswd -d www-data webshare 2>/dev/null || true; \
		$(call mk_log,info,removing webshare group); \
		sudo groupdel webshare 2>/dev/null || true; \
		sudo $(NGINX) -t; \
		sudo $(NGINX) -s reload; \
		$(call mk_log,info,removing tmp data); \
		rm -rf /var/tmp/ros2_lidar_georeference \
		$(call mk_log,succ,uninstall complete. you may need to log out/in for group changes to take effect); \
	else \
		$(call mk_log,warn,cleaning local environment); \
		docker image rm ros2_lidar_georeference:builder 2>/dev/null || true; \
		rm -rf install/ upload.zip; \
	fi