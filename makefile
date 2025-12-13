# === CONFIG ===
NGINX_SITES_ENABLED := /etc/nginx/sites-enabled
SITE_NAME := ros2_lidar_georeference
PORT := 8888
ROOT_DIR := $(shell ros2 pkg prefix ros2_lidar_georeference)
NGINX := nginx
CONF_FILE := $(NGINX_SITES_ENABLED)/$(SITE_NAME)
DATE := $(shell date +%Y%m%d%H%M%S)
WEB_ROOT := /var/www/$(SITE_NAME)
DOWNLOAD_DIR := $(WEB_ROOT)/downloads

# === RULES FOR EXECUTION ON LEO ROVER ===
nginx-install: FORCE;
	@echo "Installing nginx site $(SITE_NAME) on port $(PORT)"
	@sudo mkdir -p $(WEB_ROOT)
	@sudo mkdir -p $(DOWNLOAD_DIR)
	@sudo cp -r $(ROOT_DIR)/share/ros2_lidar_georeference/web/* $(WEB_ROOT)
	@sudo groupadd -f webshare
	@sudo usermod -a -G webshare pi
	@sudo usermod -a -G webshare www-data
	@sudo chown -R www-data:webshare $(WEB_ROOT)
	@sudo chmod -R 775 $(DOWNLOAD_DIR)
	@sudo chmod g+s $(DOWNLOAD_DIR)
	@printf "%s\n" \
		"server {" \
		"    listen $(PORT);" \
		"    listen [::]:$(PORT);" \
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
		| sudo tee $(CONF_FILE) > /dev/null
	@$(MAKE) nginx-test
	@$(MAKE) nginx-reload
	@echo "Done. Available at http://10.0.0.1:$(PORT)"
	@echo "App should write files to: $(DOWNLOAD_DIR)"

nginx-uninstall: FORCE
	@echo "Uninstalling nginx site $(SITE_NAME)"
	@if [ -f $(CONF_FILE) ]; then \
		echo "Removing nginx config..."; \
		sudo rm -f $(CONF_FILE); \
	else \
		echo "Config file not found, skipping..."; \
	fi
	@if [ -d $(WEB_ROOT) ]; then \
		echo "Removing web root $(WEB_ROOT)..."; \
		sudo rm -rf $(WEB_ROOT); \
	else \
		echo "Web root not found, skipping..."; \
	fi
	@echo "Removing users from webshare group..."
	@sudo gpasswd -d pi webshare 2>/dev/null || true
	@sudo gpasswd -d www-data webshare 2>/dev/null || true
	@echo "Removing webshare group..."
	@sudo groupdel webshare 2>/dev/null || true
	@$(MAKE) nginx-test || true
	@$(MAKE) nginx-reload
	@echo "Uninstall complete. You may need to log out/in for group changes to take effect."

nginx-reload: FORCE
	@echo "Reloading nginx"
	@sudo $(NGINX) -s reload

nginx-test: FORCE
	@echo "Testing nginx config"
	@sudo $(NGINX) -t

FORCE: ;

run: nginx-install
	@bash -c "source /opt/ros/jazzy/setup.bash && \
	    source install/setup.bash && \
	    ros2 launch ros2_lidar_georeference file_manager_with_rosbridge.launch.py"

# === RULES FOR EXECUTION ON BUILD PC ===
build-packages:
	sudo rm -rf install/
	sudo docker build --platform linux/arm64 --tag ros2_lidar_georeference:builder .
	container_id=$$(sudo docker create ros2_lidar_georeference:builder) && \
	sudo docker cp $$container_id:/ws/install/ install/ && \
	sudo docker rm $$container_id

push-packages:
	@echo "Pushing packages to Leo Rover"
	scp -r {install/,makefile} pi@10.0.0.1:/home/pi/ros2_lidar_georeference