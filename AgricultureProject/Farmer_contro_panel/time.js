function updateDateTime() {
    const now = new Date();

    const date = now.toLocaleDateString();  
    const time = now.toLocaleTimeString(); 

    document.getElementById('weather_date').textContent = `📅 Date: ${date}`;
    document.getElementById('weather_time').textContent = `🕒 Time: ${time}`;
}


updateDateTime();
setInterval(updateDateTime, 1000);
