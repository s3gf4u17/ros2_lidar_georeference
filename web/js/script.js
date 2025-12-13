$(function() {
      // === ROSBRIDGE CONNECTION ===
      const ros = new ROSLIB.Ros({
        url: 'ws://' + window.location.hostname + ':9091'
      });

      ros.on('connection', function() {
        $('#status').text('Connected to ROS bridge');
      });

      ros.on('error', function(error) {
        $('#error').text('Error connecting: ' + error);
      });

      ros.on('close', function() {
        $('#status').text('Disconnected from ROS bridge');
      });

      // === BATTERY MONITOR ===
      const batteryTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/firmware/battery',
        messageType: 'std_msgs/Float32'
      });

      batteryTopic.subscribe(function(message) {
        $('#battery-value').text(message.data.toFixed(2));
      });

      const deleteService = new ROSLIB.Service({
        ros: ros,
        name: '/thesis/delete_file',
        serviceType: 'ros2_lidar_georeference/srv/FileDelete'
      });

      // === LIDAR CONFIGS ===
      const configsTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/thesis/configs',
        messageType: 'ros2_lidar_georeference/msg/FileList'
      });

      configsTopic.subscribe((msg) => {
        try {
          const configs = JSON.parse(msg.json_list);
          renderConfigs(configs);
        } catch (e) {
          console.error('Failed to parse config JSON:', e);
        }
      });



      function renderConfigs(configs) {
  const container = $('#configs');
  container.empty();

  if (!configs || configs.length === 0) {
    container.html('<p>No configuration files found.</p>');
    return;
  }

  configs.forEach(cfg => {
    const card = $(`
      <div class="config-card">
        <div class="uuid">${cfg.uuid}</div>
        <div class="meta"><strong>Author:</strong> ${cfg.author || '—'}</div>
        <div class="meta"><strong>Timestamp:</strong> ${new Date(cfg.timestamp).toLocaleString()}</div>

        <a class="download-link" href="${cfg.download}" download>
          ⬇ Download File
        </a>

        <button class="delete-btn">🗑 Delete</button>
      </div>
    `);

    // DELETE BUTTON HANDLER
    card.find('.delete-btn').on('click', () => {
      if (!confirm(`Delete configuration ${cfg.uuid}?`)) {
        return;
      }

      const request = new ROSLIB.ServiceRequest({
        uuid: cfg.uuid
      });

      deleteService.callService(
        request,
        (response) => {
          if (response.response_value === 0) {
            alert('Configuration deleted successfully.');
          } else {
            alert('Failed to delete configuration.');
          }
        },
        (error) => {
          console.error('Service call failed:', error);
          alert('Error calling delete service.');
        }
      );
    });

    container.append(card);
  });
}



    });