const blightListener = new ROSLIB.Topic({
    ros: ros,
    name: '/blight_detection',
    messageType: 'std_msgs/msg/String'
});

const blightWarningsDiv = document.getElementById('blightWarnings');
let blightDetections = [];

function updateBlightWarnings() {

    blightWarningsDiv.querySelectorAll('p').forEach(el => el.remove());


    const count = blightDetections.length;
    const countP = document.createElement('p');
    countP.textContent = 'Blight count (last 24h): ' + count;
    blightWarningsDiv.appendChild(countP);


    blightDetections.slice().reverse().forEach(timestamp => {
const p = document.createElement('p');
p.style.backgroundColor = 'red';
p.style.color = 'white';

const dt = new Date(timestamp);  
const formatted = dt.toLocaleString('en-GB', {
    year: '2-digit',
    month: '2-digit',
    day: '2-digit',
    hour: '2-digit',
    minute: '2-digit'
});

p.textContent = `Blight detected! : ${formatted}`;
blightWarningsDiv.appendChild(p);


    });
}

function pruneOldDetections() {
    const now = new Date();
    blightDetections = blightDetections.filter(t => {
        const dt = new Date(t);
        return now - dt <= 24 * 60 * 60 * 1000; // Keep detections within last 24h
    });
    updateBlightWarnings();
}

console.log("Total blights stored:", blightDetections.length);

blightListener.subscribe(function (message) {
    console.log("Received blight message:", message.data);
    if (message.data.trim().toLowerCase() === "blight") {
const now = new Date();
const isoTimestamp = now.toISOString(); 
blightDetections.push(isoTimestamp);
pruneOldDetections();
}

});
