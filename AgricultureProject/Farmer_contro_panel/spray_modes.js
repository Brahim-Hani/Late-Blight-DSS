const sprayAcceptPublisher = new ROSLIB.Topic({
    ros: ros,
    name: 'spray_accept',
    messageType: 'std_msgs/String'
});

const sprayCancelPublisher = new ROSLIB.Topic({
    ros: ros,
    name: 'spray_cancel',
    messageType: 'std_msgs/String'
});

const sprayingModePublisher = new ROSLIB.Topic({
    ros: ros,
    name: 'spraying_mode',
    messageType: 'std_msgs/String'
});

// ROS Listeners
const sprayStatusListener = new ROSLIB.Topic({
    ros: ros,
    name: 'spray_status',
    messageType: 'std_msgs/String'
});

const sprayScheduleNotifyListener = new ROSLIB.Topic({
    ros: ros,
    name: 'spray_notification',
    messageType: 'std_msgs/String'
});

let sprayPromptDiv = null;
let currentMode = 'manual'; // Track mode

// Function to publish and update mode
function sendModeToROS(isAuto) {
    currentMode = isAuto ? 'auto' : 'manual';
    const msg = new ROSLIB.Message({ data: currentMode });
    sprayingModePublisher.publish(msg);

    // Update UI
    const modeDisplay = document.getElementById('sprayModeDisplay');
    if (modeDisplay) {
        modeDisplay.textContent = currentMode;
    }
}

// Create prompt inside spray-control-container
function createSprayPrompt(dateTime, mode) {
    if (sprayPromptDiv) sprayPromptDiv.remove();

    sprayPromptDiv = document.createElement('div');
    sprayPromptDiv.classList.add('spray-prompt');

    const message = document.createElement('p');
    message.textContent = mode === 'auto'
        ? `✅ Spray scheduled at ${dateTime}`
        : `🤖 Robot suggests spraying at ${dateTime}`;
    sprayPromptDiv.appendChild(message);

    const cancelBtn = document.createElement('button');
    cancelBtn.textContent = 'Cancel';
    cancelBtn.onclick = () => {
        sprayCancelPublisher.publish(new ROSLIB.Message({ data: 'cancel' }));
        sprayPromptDiv.remove();
    };
    sprayPromptDiv.appendChild(cancelBtn);

    if (mode === 'manual') {
        const acceptBtn = document.createElement('button');
        acceptBtn.textContent = 'Accept';
        acceptBtn.onclick = () => {
            sprayAcceptPublisher.publish(new ROSLIB.Message({ data: 'accept' }));
            acceptBtn.remove(); 
        };
        sprayPromptDiv.appendChild(acceptBtn);
    }

    const container = document.getElementById('spray-control-container');
    if (container) {
        container.appendChild(sprayPromptDiv);
    } else {
        document.body.appendChild(sprayPromptDiv); 
    }
}


sprayStatusListener.subscribe(function (message) {
    const msg = message.data.toLowerCase();
    if (msg === 'spray cancelled' || msg === 'spraying ended' || msg === 'spray expired') {
        if (sprayPromptDiv) sprayPromptDiv.remove();
    }
});


sprayScheduleNotifyListener.subscribe(function (message) {
    const msg = message.data;
    const lowerMsg = msg.toLowerCase();
    if (lowerMsg.includes('spray scheduled at') || lowerMsg.includes('spray suggested at')) {
        const dateTime = msg.split(' at ')[1];
        createSprayPrompt(dateTime, currentMode);
    }
});


document.addEventListener('DOMContentLoaded', () => {
    const toggleInput = document.querySelector('.spray_switch input');
    if (toggleInput) {
        
        sendModeToROS(toggleInput.checked);

        
        toggleInput.addEventListener('change', function () {
            sendModeToROS(this.checked);
        });
    }
});
