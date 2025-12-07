import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from datetime import datetime
from dateutil import parser as date_parser

class SprayDecider(Node):
    def __init__(self):
        super().__init__('spray_decider')

        # Publishers
        self.status_pub = self.create_publisher(String, 'spray_status', 10)
        self.log_pub = self.create_publisher(String, 'spray_log', 10)
        self.schedule_notify_pub = self.create_publisher(String, 'spray_notification', 10)  # Notification publisher

        # Subscribers
        self.command_sub = self.create_subscription(String, 'spray_command', self.command_callback, 10)
        self.schedule_sub = self.create_subscription(String, 'spray_schedule', self.schedule_callback, 10)
        self.mode_sub = self.create_subscription(String, 'spraying_mode', self.mode_callback, 10)
        self.accept_sub = self.create_subscription(String, 'spray_accept', self.user_response_callback, 10)
        self.cancel_sub = self.create_subscription(String, 'spray_cancel', self.cancel_callback, 10)

        # Internal state
        self.is_spraying = False
        self.spray_end_timer = None
        self.scheduled_time = None
        self.schedule_timer = None
        self.mode = 'manual'  # 'auto' or 'manual'
        self.accepted_by_user = False

        self.get_logger().info('✅ Spray Decider Node is up and running.')

    def command_callback(self, msg):
        if msg.data.lower() == 'spray':
            if not self.is_spraying:
                self.start_spraying()
            else:
                self.get_logger().info('⚠️ Already spraying. Ignoring duplicate command.')

    def schedule_callback(self, msg):
        try:
            self.scheduled_time = date_parser.parse(msg.data)
            self.accepted_by_user = (self.mode == 'auto')
            now = datetime.now()

            if self.scheduled_time <= now:
                self.get_logger().warn('⚠️ Scheduled time is in the past. Ignoring.')
                return

            delay = (self.scheduled_time - now).total_seconds()
            if self.schedule_timer:
                self.schedule_timer.cancel()

            self.schedule_timer = self.create_timer(delay, self.scheduled_spray_callback)

            self.get_logger().info(f"🗓️ Received schedule for: {self.scheduled_time.isoformat()}")
            if self.mode == 'manual':
                self.get_logger().info("🧍 Awaiting user approval (manual mode).")
            else:
                self.get_logger().info("🤖 Auto mode: schedule accepted.")

            # Publish schedule notification
            notification_msg = f"Spray scheduled at {self.scheduled_time.isoformat()}" if self.mode == 'auto' else f"Spray suggested at {self.scheduled_time.isoformat()}"
            self.schedule_notify_pub.publish(String(data=notification_msg))

        except Exception as e:
            self.get_logger().error(f"❌ Failed to parse scheduled time: {e}")

    def mode_callback(self, msg):
        self.mode = msg.data.strip().lower()
        self.get_logger().info(f"⚙️ Spraying mode set to: {self.mode}")

    def user_response_callback(self, msg):
        if self.mode != 'manual':
            return
        response = msg.data.strip().lower()
        if response == 'accept':
            if self.scheduled_time:
                self.accepted_by_user = True
                self.get_logger().info("✅ User accepted the scheduled spray.")
                self.schedule_notify_pub.publish(String(data=f"✅ Spray officially scheduled for {self.scheduled_time.isoformat()}"))
            else:
                self.get_logger().warn("⚠️ No scheduled spray to accept.")
        elif response == 'decline':
            self.get_logger().info("🚫 User declined the scheduled spray.")
            self.cancel_callback(msg)

    def cancel_callback(self, msg):
        if self.scheduled_time:
            self.get_logger().info("🚫 Scheduled spray cancelled.")
            self.status_pub.publish(String(data="spray cancelled"))

            # Publish cancel notification
            cancel_time = datetime.now().isoformat()
            self.schedule_notify_pub.publish(String(data=f"❌ Spray canceled at {cancel_time}"))

            self.scheduled_time = None
            self.accepted_by_user = False
            if self.schedule_timer:
                self.schedule_timer.cancel()
                self.schedule_timer = None

    def scheduled_spray_callback(self):
        if self.scheduled_time and (self.mode == 'auto' or self.accepted_by_user):
            self.get_logger().info("⏰ Scheduled time reached. Executing spray.")
            self.start_spraying()
        else:
            self.get_logger().info("❌ Scheduled time reached but spray not accepted. Skipping.")

        # Reset schedule
        self.scheduled_time = None
        self.accepted_by_user = False
        if self.schedule_timer:
            self.schedule_timer.cancel()
            self.schedule_timer = None

    def start_spraying(self):
        self.is_spraying = True
        self.status_pub.publish(String(data='spraying now'))
        self.get_logger().info('🚿 Started spraying.')

        timestamp = datetime.now().isoformat()
        self.log_pub.publish(String(data=timestamp))
        self.get_logger().info(f'📝 Spray log published: {timestamp}')

        self.spray_end_timer = self.create_timer(60.0, self.end_spraying)

    def end_spraying(self):
        if self.is_spraying:
            self.status_pub.publish(String(data='spraying ended'))
            self.get_logger().info('🛑 Ended spraying.')
            self.is_spraying = False
            self.spray_end_timer.cancel()
            self.spray_end_timer = None

def main(args=None):
    rclpy.init(args=args)
    node = SprayDecider()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

