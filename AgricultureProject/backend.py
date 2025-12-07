from flask import Flask, request, jsonify
from flask_cors import CORS
import subprocess
import os
import signal
import psutil

app = Flask(__name__)
CORS(app)

ros_process = None  # Global process handle

@app.route('/start', methods=['POST'])
def start_ros():
    global ros_process
    if ros_process is None:
        env = os.environ.copy()
        env_script = '/home/hixy/ros2_blight_ws/install/setup.bash'  # Adjust as needed

        cmd = f"bash -c 'source {env_script} && ros2 launch spray_scheduler_pkg system_launcher.launch.py'"
        ros_process = subprocess.Popen(
            cmd,
            shell=True,
            preexec_fn=os.setsid,
            env=env
        )
        return jsonify({"status": "started"}), 200
    else:
        return jsonify({"status": "already running"}), 200

@app.route('/stop', methods=['POST'])
def stop_ros():
    global ros_process
    if ros_process:
        try:
            parent = psutil.Process(ros_process.pid)
            for child in parent.children(recursive=True):
                child.kill()
            parent.kill()
        except Exception as e:
            print(f"Error stopping ROS process: {e}")
        ros_process = None
        return jsonify({"status": "stopped"}), 200
    else:
        return jsonify({"status": "not running"}), 200

@app.route('/status', methods=['GET'])
def ros_status():
    global ros_process
    if ros_process and ros_process.poll() is None:
        return jsonify({"status": "started"}), 200
    return jsonify({"status": "not running"}), 200

if __name__ == '__main__':
    print("ROS system launcher started. Waiting for web trigger...")
    app.run(host='0.0.0.0', port=5000)
