const weatherListener = new ROSLIB.Topic({
    ros: ros,
    name: '/weather_status',
    messageType: 'std_msgs/String'
});

weatherListener.subscribe(function (message) {
    try {
        const data = JSON.parse(message.data);

        const city = data.city;
        const now = data.now;
        const nextBlightTime = data.next_blight_suitable_time;
        const nextSprayTime = data.next_spray_suitable_time;
        const forecast = data.forecast_24h;

        const getEl = id => document.getElementById(id);

        
        if (getEl('city')) getEl('city').textContent = city;
        if (getEl('current_temp')) getEl('current_temp').textContent = `${now.temperature} °C`;
        if (getEl('current_humidity')) getEl('current_humidity').textContent = `${now.humidity} %`;
        if (getEl('current_wind')) getEl('current_wind').textContent = `${now.wind} m/s`;

        if (getEl('current_status')) {
            if (nextSprayTime !== "none") {
                const nowTime = new Date();
                const sprayTime = new Date(nextSprayTime);
                const hoursUntilSpray = Math.round((sprayTime - nowTime) / (1000 * 60 * 60));
                getEl('current_status').textContent = `Next Suitable Spraying Conditions in ${hoursUntilSpray} hour(s)`;
            } else {
                getEl('current_status').textContent = "No sprayable window in next 24h";
            }
        }

        
        let suitableIn = "none";
        let future = null;

        if (nextBlightTime !== "none") {
            const nowTime = new Date();
            const targetTime = new Date(nextBlightTime);
            suitableIn = Math.round((targetTime - nowTime) / (1000 * 60 * 60)); // hours
            future = forecast.find(f => f.time === nextBlightTime);
        }

        if (getEl('future_temp')) {
            getEl('future_temp').textContent = future ? `${future.temperature} °C` : "N/A";
        }
        if (getEl('future_humidity')) {
            getEl('future_humidity').textContent = future ? `${future.humidity} %` : "N/A";
        }
        if (getEl('future_status')) {
            getEl('future_status').textContent = future && future.blight_suitable ? "suitable" : "not suitable";
        }

        if (getEl('suitable_in')) {
            getEl('suitable_in').textContent =
                suitableIn === "none" ? "No favorable conditions in next 24h" : `Favorable in ${suitableIn} hour(s)`;
        }

        // Warnings
        const warningsDiv = getEl('warnings-list');
        if (warningsDiv) {
            const warning = document.createElement('p');
            warning.style.textAlign = 'center';
            warning.style.color = 'white';
            warning.style.margin = '3px 0';

            if (now.blight_suitable) {
                warning.textContent = `Favorable blight conditions: Now`;
                warning.style.backgroundColor = 'red';
            } else if (suitableIn !== "none") {
                warning.textContent = `Favorable in ${suitableIn} hour(s)`;
                warning.style.backgroundColor = 'red';
            } else {
                warning.textContent = `✅ No favorable conditions in next 24h`;
                warning.style.backgroundColor = 'green';
            }

            warningsDiv.innerHTML = '';
            warningsDiv.appendChild(warning);
        }

        console.log('[ROS] Weather data and forecast updated.');

    } catch (err) {
        console.error('[ROS] Failed to parse weather data:', err);
    }
});
