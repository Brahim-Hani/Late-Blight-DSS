import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
from datetime import datetime, timedelta

class SprayScheduler(Node):
    def __init__(self):
        super().__init__('spray_scheduler')

        # Subscriptions
        self.weather_sub = self.create_subscription(String, 'weather_status', self.weather_callback, 10)
        self.blight_sub = self.create_subscription(String, 'blight_detection', self.blight_callback, 10)
        self.log_sub = self.create_subscription(String, 'spray_log', self.log_callback, 10)
        self.cancel_sub = self.create_subscription(String, 'spray_cancel', self.cancel_callback, 10)

        # Publishers
        self.schedule_pub = self.create_publisher(String, 'spray_schedule', 10)
        self.cancel_pub = self.create_publisher(String, 'spray_cancel', 10)

        # Internal state
        self.latest_weather = None
        self.blight_detected = False
        self.current_scheduled_time = None
        self.last_spray_time = None

        self.expiry_timer = self.create_timer(60.0, self.check_schedule_expiry)

        self.get_logger().info("📅 Spray Scheduler is running.")

    def weather_callback(self, msg):
        try:
            self.latest_weather = json.loads(msg.data)
            self.get_logger().info("✅ Received weather status.")
            self.evaluate_spray_need()
        except Exception as e:
            self.get_logger().error(f"❌ Failed to parse weather_status: {e}")

    def blight_callback(self, msg):
        try:
            if msg.data.strip().lower() == "blight":
                self.blight_detected = True
                self.get_logger().info("⚠️ Blight detected by model.")
                self.evaluate_spray_need()
        except Exception as e:
            self.get_logger().error(f"❌ Failed to parse blight_detection: {e}")

    def log_callback(self, msg):
        try:
            log_time = datetime.fromisoformat(msg.data.strip())
            self.last_spray_time = log_time
            self.get_logger().info(f"📘 Spray completed at {log_time.isoformat()}")
            self.clear_schedule("🧹 Spray completed.")
        except Exception as e:
            self.get_logger().warn(f"⚠️ Invalid spray log: {msg.data}")

    def cancel_callback(self, msg):
        self.get_logger().info(f"🚫 Spray canceled: {msg.data.strip()}")
        self.clear_schedule("🧹 Spray canceled.")

    def clear_schedule(self, reason):
        if self.current_scheduled_time:
            self.get_logger().info(f"{reason} Clearing schedule at: {self.current_scheduled_time}")
            self.current_scheduled_time = None

    def check_schedule_expiry(self):
        if not self.current_scheduled_time:
            return
        try:
            scheduled_dt = datetime.fromisoformat(self.current_scheduled_time)
            if scheduled_dt < datetime.now():
                self.get_logger().warn(f"⏰ Scheduled spray at {self.current_scheduled_time} has expired.")
                self.cancel_pub.publish(String(data='cancel'))
                self.clear_schedule("❌ Schedule expired.")
        except Exception as e:
            self.get_logger().error(f"❌ Failed to parse scheduled time: {e}")

    def evaluate_spray_need(self):
        if not self.latest_weather or self.current_scheduled_time:
            return

        next_spray_time = self.latest_weather.get("next_spray_suitable_time", "none")
        next_blight_time = self.latest_weather.get("next_blight_suitable_time", "none")
        should_schedule = False

        if self.blight_detected:
            should_schedule = True
        elif next_blight_time != "none":
            self.get_logger().info("🌫️ Blight-favorable forecast detected.")
            should_schedule = True
        else:
            self.get_logger().info("✅ No blight detection or forecast.")

        if should_schedule and next_spray_time != "none":
            if self.last_spray_time and (datetime.now() - self.last_spray_time).days < 7:
                self.get_logger().info("⛔ No spray scheduled — sprayed recently.")
                return

            try:
                next_time_obj = datetime.fromisoformat(next_spray_time)
                if next_time_obj.hour == datetime.now().hour and next_time_obj.date() == datetime.now().date():
                    self.get_logger().info("⏳ Skipping current hour for scheduling.")
                    return
            except Exception as e:
                self.get_logger().warn(f"⚠️ Couldn't parse next_spray_time: {next_spray_time}")
                return

            self.get_logger().info(f"🗓️ Scheduling spray at: {next_spray_time}")
            self.current_scheduled_time = next_spray_time
            self.schedule_pub.publish(String(data=next_spray_time))
        else:
            self.get_logger().info("🛑 No spray scheduled.")


def main(args=None):
    rclpy.init(args=args)
    node = SprayScheduler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


