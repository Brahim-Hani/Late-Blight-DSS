

// Connect to ROS topic
const notificationListener = new ROSLIB.Topic({
    ros: ros,
    name: '/spray_notification',
    messageType: 'std_msgs/String'
  });
  
  // When a new notification arrives
  notificationListener.subscribe(function (message) {
    const notifContainer = document.getElementById('notifications-list');
    const now = new Date().toLocaleTimeString();
  
    const div = document.createElement('div');
    div.className = 'notification';
    div.innerText = `[${now}] ${message.data}`;
    
    notifContainer.prepend(div);
  
    // Keep only last 20 notifications
    if (notifContainer.children.length > 20) {
      notifContainer.removeChild(notifContainer.lastChild);
    }
  });
  