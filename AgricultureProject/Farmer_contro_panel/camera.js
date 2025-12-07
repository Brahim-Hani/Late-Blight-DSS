
  const controlTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/blight_detector/control',
    messageType: 'std_msgs/String'
  });
  
  const blightSwitch = document.querySelector('.blight_switch input');
  
  blightSwitch.addEventListener('change', function () {
    const command = this.checked ? 'resume' : 'pause';
    controlTopic.publish(new ROSLIB.Message({ data: command }));
    console.log(`📤 Sent: ${command}`);
  });
  
