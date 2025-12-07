let ros = new ROSLIB.Ros({
  url: 'ws://localhost:9090'
});

ros.on('connection', () => {
  console.log(' Connected to rosbridge WebSocket server.');
});

ros.on('error', (error) => {
  console.error(' Error connecting to rosbridge:', error);
});

ros.on('close', () => {
  console.warn(' Connection to rosbridge closed.');
});
