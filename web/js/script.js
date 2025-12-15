$(function () {
  const ws = new WebSocket('ws://' + window.location.hostname + ':9091');
  let msgId = 0;
  let measuring = false;

  const $btn = $('#action-button');
  const $feedback = $('#feedback-container');
  const $result = $('#result-container');

  function send(op, data) {
    ws.send(JSON.stringify({
      op: op,
      id: 'msg_' + (++msgId),
      ...data
    }));
  }

  ws.onopen = () => {
    $('#status').text('Connected');
    $btn.prop('disabled', false);

    // Battery
    send('subscribe', {
      topic: '/firmware/battery',
      type: 'std_msgs/Float32'
    });

    // Config list
    send('subscribe', {
      topic: '/thesis/configs',
      type: 'ros2_lidar_georeference/msg/FileList'
    });

    // Measurement feedback (WEB BRIDGE)
    send('subscribe', {
      topic: '/measurement/web/feedback',
      type: 'std_msgs/Int32'
    });
  };

  ws.onclose = () => {
    $('#status').text('Disconnected');
    $btn.prop('disabled', true);
  };

  ws.onerror = (e) => {
    $('#error').text('WebSocket error');
    console.error(e);
  };

  ws.onmessage = (event) => {
    const msg = JSON.parse(event.data);

    if (msg.op !== 'publish') return;

    if (msg.topic === '/measurement/web/feedback') {
      updateFeedback(msg.msg.data);
    }

    if (msg.topic === '/firmware/battery') {
      $('#battery-value').text(msg.msg.data.toFixed(2));
    }

    if (msg.topic === '/thesis/configs') {
      try {
        renderConfigs(JSON.parse(msg.msg.json_list));
      } catch {}
    }
  };

  function updateFeedback(state) {
    const map = {
      0: 'Idle',
      1: 'Collecting',
      2: 'Processing'
    };
    $feedback.html(`<strong>Status:</strong> ${map[state] || 'Unknown'}`);

    if (state === 0 && measuring) {
      measuring = false;
      $btn.text('Start Measurement').removeClass('stop').addClass('start');
    }
  }

  function startMeasurement() {
    send('publish', {
      topic: '/measurement/web/goal',
      type: 'std_msgs/Int32',
      msg: { data: 1 }
    });
    measuring = true;
    $btn.text('Stop & Process').removeClass('start').addClass('stop');
    $result.empty();
  }

  function stopMeasurement() {
    send('publish', {
      topic: '/measurement/web/result',
      type: 'std_msgs/Int32',
      msg: { data: 1 }
    });
  }

  $btn.on('click', () => {
    measuring ? stopMeasurement() : startMeasurement();
  });

  function renderConfigs(configs) {
    const c = $('#configs').empty();
    if (!configs.length) return c.html('<p>No configs</p>');

    configs.forEach(cfg => {
      c.append(`
        <div class="config-card">
          <div class="uuid">${cfg.uuid}</div>
          <div>${new Date(cfg.timestamp).toLocaleString()}</div>
          <a href="${cfg.download}" download>⬇ Download</a>
        </div>
      `);
    });
  }
});
