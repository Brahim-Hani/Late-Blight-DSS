import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import requests
import json
from datetime import datetime
from dateutil import tz

API_KEY = "605cf761688fa2c65de8db87e70ad829"
CITY = "Blida"
LAT = 36.47
LON = 2.83

OWM_URL = f"http://api.openweathermap.org/data/2.5/weather?q={CITY},DZ&appid={API_KEY}&units=metric"
OPEN_METEO_FORECAST_URL = (
    f"https://api.open-meteo.com/v1/forecast?"
    f"latitude={LAT}&longitude={LON}"
    f"&hourly=temperature_2m,relative_humidity_2m,precipitation,wind_speed_10m"
    f"&forecast_days=2"
    f"&timezone=auto"
)

def is_blight_favorable(temp, humidity):
    return 10 <= temp <= 25 and humidity > 90

def is_sprayable(temp, rain, wind):
    return 5 < temp < 25 and rain == 0 and wind < 10

class WeatherReporter(Node):
    def __init__(self):
        super().__init__('weather_reporter')
        self.publisher_ = self.create_publisher(String, 'weather_status', 10)
        self.timer = self.create_timer(600.0, self.check_weather)
        self.check_weather()

    def check_weather(self):
        try:
            # Get current weather from OpenWeatherMap
            owm_response = requests.get(OWM_URL)
            owm_response.raise_for_status()
            owm_data = owm_response.json()
            temp_now = owm_data["main"]["temp"]
            wind_now = owm_data["wind"]["speed"]
            rain_now = 0  

            # Get forecast from Open-Meteo
            forecast_response = requests.get(OPEN_METEO_FORECAST_URL)
            forecast_response.raise_for_status()
            forecast_data = forecast_response.json()

            hourly_times = forecast_data["hourly"]["time"]
            hourly_temps = forecast_data["hourly"]["temperature_2m"]
            hourly_humidities = forecast_data["hourly"]["relative_humidity_2m"]
            hourly_precip = forecast_data["hourly"]["precipitation"]
            hourly_winds = forecast_data["hourly"]["wind_speed_10m"]

            now = datetime.now(tz=tz.tzlocal())
            now_str = now.strftime('%Y-%m-%dT%H:00')

            try:
                start_index = hourly_times.index(now_str)
            except ValueError:
                self.get_logger().error(f"Current time {now_str} not found in forecast.")
                return

            next_blight = None
            next_spray = None
            forecast_24h = []

            
            for i in range(start_index + 1, min(start_index + 25, len(hourly_times))):
                temp = hourly_temps[i]
                humidity = hourly_humidities[i]
                rain = hourly_precip[i]
                wind = hourly_winds[i]
                time_str = hourly_times[i]

                if next_blight is None and is_blight_favorable(temp, humidity):
                    next_blight = time_str

                if next_spray is None and is_sprayable(temp, rain, wind):
                    next_spray = time_str

                forecast_24h.append({
                    "time": time_str,
                    "temperature": temp,
                    "humidity": humidity,
                    "rain": rain,
                    "wind": wind,
                    "blight_suitable": is_blight_favorable(temp, humidity),
                    "spray_suitable": is_sprayable(temp, rain, wind)
                })


            current_humidity = None
            for i in range(start_index, min(start_index + 2, len(hourly_humidities))):
                current_humidity = hourly_humidities[i]
                if current_humidity is not None:
                    break

            current_blight = is_blight_favorable(temp_now, current_humidity if current_humidity else 0)

            data_to_publish = {
                "city": CITY,
                "now": {
                    "temperature": temp_now,
                    "humidity": current_humidity,
                    "wind": wind_now,
                    "blight_suitable": current_blight
                },
                "next_blight_suitable_time": next_blight if next_blight else "none",
                "next_spray_suitable_time": next_spray if next_spray else "none",
                "forecast_24h": forecast_24h
            }

            msg = String()
            msg.data = json.dumps(data_to_publish)
            self.publisher_.publish(msg)
            self.get_logger().info(
                f"📤 Published summary to /weather_status | "
                f"Next blight: {next_blight} | Next spray: {next_spray}"
            )

        except Exception as e:
            self.get_logger().error(f"❌ Exception during weather fetch: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = WeatherReporter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

