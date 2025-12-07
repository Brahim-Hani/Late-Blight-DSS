import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ultralytics import YOLO
import cv2
import base64
import os

class BlightDetector(Node):
    def __init__(self):
        super().__init__('blight_detector')

        self.publisher_ = self.create_publisher(String, 'blight_detection', 10)
        self.image_publisher_ = self.create_publisher(String, 'blight_image', 10)

        self.control_subscriber_ = self.create_subscription(
            String,
            'blight_detector/control',
            self.control_callback,
            10
        )

        self.paused = True  
        self.cap = None
        self.window_name = "Blight Detection - Camera Feed"
        self.window_open = False
        self.blight_already_published = False

        self.get_logger().info("Loading YOLOv8 model...")
        try:
            model_path = os.path.join(os.path.dirname(__file__), "best.pt")
            self.model = YOLO(model_path)
            self.get_logger().info("✅ Model loaded successfully.")
        except Exception as e:
            self.get_logger().error(f"❌ Error loading model: {e}")
            return

        self.timer = self.create_timer(0.1, self.detect_blight)

    def control_callback(self, msg: String):
        command = msg.data.strip().lower()
        if command == "pause":
            if not self.paused:
                self.paused = True
                self.release_camera()
                self.get_logger().info("⏸️ Blight detection paused.")
        elif command == "resume":
            if self.paused:
                self.paused = False
                self.initialize_camera()
                self.get_logger().info("▶️ Blight detection resumed.")
        else:
            self.get_logger().warn(f"⚠️ Unknown control command: {msg.data}")

    def initialize_camera(self):
        if self.cap is None or not self.cap.isOpened():
            self.cap = cv2.VideoCapture(0)
            if not self.cap.isOpened():
                self.get_logger().error("❌ Failed to open webcam.")
                return
            else:
                self.get_logger().info("✅ Webcam opened successfully.")
        self.window_open = True

    def release_camera(self):
        if self.cap is not None:
            self.cap.release()
            self.cap = None
        if self.window_open:
            cv2.destroyWindow(self.window_name)
            self.window_open = False

    def is_inside(self, inner, outer):
        return (
            inner[0] >= outer[0] and
            inner[1] >= outer[1] and
            inner[2] <= outer[2] and
            inner[3] <= outer[3]
        )

    def detect_blight(self):
        if self.paused or self.cap is None:
            return

        ret, frame = self.cap.read()
        if not ret or frame is None:
            self.get_logger().warn("⚠️ Could not read frame.")
            return

        try:
            results = self.model.predict(source=frame, conf=0.5, verbose=False)
            boxes = results[0].boxes
            box_coords = boxes.xyxy.tolist()
            class_ids = boxes.cls.tolist()

            leaves = []
            blights = []

            for i, class_id in enumerate(class_ids):
                name = self.model.names[int(class_id)]
                if name == "leaf":
                    leaves.append(box_coords[i])
                elif name == "blight":
                    blights.append(box_coords[i])

            annotated_frame = frame.copy()

            
            for box in leaves:
                x1, y1, x2, y2 = map(int, box)
                cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(annotated_frame, "leaf", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            
            blight_inside_leaf = False
            for blight_box in blights:
                for leaf_box in leaves:
                    if self.is_inside(blight_box, leaf_box):
                        x1, y1, x2, y2 = map(int, blight_box)
                        cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
                        cv2.putText(annotated_frame, "blight", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                        blight_inside_leaf = True
                        break
                if blight_inside_leaf:
                    break

          
            try:
                cv2.imshow(self.window_name, annotated_frame)
                if cv2.getWindowProperty(self.window_name, cv2.WND_PROP_VISIBLE) < 1:
                    self.get_logger().warn("🚫 Camera window closed manually.")
                    self.window_open = False
                    cv2.destroyWindow(self.window_name)
            except Exception as e:
                self.get_logger().warn(f"🚫 Could not display camera window: {e}")
                self.window_open = False

            cv2.waitKey(1)


            if blight_inside_leaf and not self.blight_already_published:
                self.publisher_.publish(String(data="blight"))
                self.get_logger().info("📡 Published: blight")

                success, buffer = cv2.imencode('.jpg', frame)
                if success:
                    img_base64 = base64.b64encode(buffer).decode('utf-8')
                    self.image_publisher_.publish(String(data=img_base64))
                    self.get_logger().info("📤 Blight image published to /blight_image")
                else:
                    self.get_logger().warn("⚠️ Failed to encode image.")

                self.blight_already_published = True
            elif not blight_inside_leaf:
                self.blight_already_published = False

        except Exception as e:
            self.get_logger().error(f"❌ Error during detection: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = BlightDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.release_camera()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

