 const sprayCommandTopic = new ROSLIB.Topic({
    ros,
    name: '/spray_command',
    messageType: 'std_msgs/String'
  });

  function sendSprayCommand() {
    const msg = new ROSLIB.Message({ data: 'spray' });
    sprayCommandTopic.publish(msg);
    console.log('Spray command sent');
  }

  // Subscriber for spray status
  const sprayStatusTopic = new ROSLIB.Topic({
    ros,
    name: '/spray_status',
    messageType: 'std_msgs/String'
  });

  sprayStatusTopic.subscribe((message) => {
    document.getElementById('sprayStatus').textContent = message.data;
  });

  // Subscriber for spray log (only show the latest spray event)
  const sprayLogTopic = new ROSLIB.Topic({
    ros,
    name: '/spray_log',
    messageType: 'std_msgs/String'
  });

  sprayLogTopic.subscribe((message) => {
    document.getElementById('sprayLog').textContent = message.data;
  });
