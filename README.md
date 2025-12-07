# Late-Blight-DSS-System


This repository contains the software and mechanical design files for an integrated Intelligent Decision Support System aimed at the precise management and prevention of Potato Late Blight (Phytophthora infestans)


Key Features
Our system provides a comprehensive, end-to-end solution for sustainable crop management:


- Real-Time Disease Detection: Integrates a lightweight YOLOv8n model for early and accurate visual identification of Late Blight symptoms on potato leaves.
- Predictive Risk Assessment: Combines visual detection with a real-time weather forecasting system (using external APIs) to anticipate environmental conditions conducive to disease outbreaks.
- Intelligent Decision Logic: A core decision-making module proposes and schedules optimal precision spraying actions based on a combination of real-time detection and predictive risk analysis.
- Robotic Integration: All software components are integrated onto a physical robotic platform featuring an adjustable wheelbase and a precise spraying mechanism designed for agricultural fields.
- Intuitive Web Interface: Farmers can remotely monitor crop health, receive instant alerts, evaluate risk levels, and control spraying operations via an easy-to-use web UI utilizing ROS 2 and WebSocket communication.




Technical Stack
The core technologies used in this project are: Deep Learning (YOLOv8n) for real-time object detection, ROS 2 (Robot Operating System 2) for modular communication and system integration, Flask for the web interface, WebSocket for high-speed, two-way communication, and Meteorological APIs for real-time data collection and prediction.

Performance & Results
The final YOLOv8n model demonstrated robust performance for field deployment: Global mAP@0.5 was 0.895, and the Late Blight Detection Accuracy was 0.894 on validation data. This robust performance confirms the model's reliability for identifying diseases in real agricultural environments.

Setup and Installation
This project is built as a complete ROS 2 package on a robotic platform.

Prerequisites
You need ROS 2 (specify the distribution used, e.g., Foxy, Galactic, or Humble), a Python environment with YOLOv8 Dependencies (including ultralytics, PyTorch with GPU support highly recommended), a Python environment with Flask for the web server, and network access to a reliable weather API (API key required).



Installation Steps
Clone the Repository: git clone https://github.com/Brahim-Hani/Late-Blight-DSS and navigate to the source directory cd Late-Blight-DSS/src.

Install Python Dependencies: pip install -r requirements.txt (This file should include: ultralytics, Flask, ros-web-bridge packages, etc.).

Build the ROS 2 Workspace: Go back to the workspace root cd .., then run colcon build followed by source install/setup.bash.

Configure API Keys: Update the configuration file for the weather_reporter node with your meteorological API key.

Launch the System: The entire system can be launched using a single setup script for automated deployment: python3 /path/to/flask_backend/backend.py




Future Roadmap
We have identified several key areas for future development and enhancement (as detailed in Chapter 4 of the thesis): Advanced AI (Investigate Few-shot Learning), Multi-Disease Capability, Hyperlocal Prediction (Integrate with on-site weather stations), Autonomous Navigation, and Dynamic Spraying.

Authors and Acknowledgements
This project was completed as a Master's Thesis in Embedded Systems Electronics. Presented by: LARBI AISSA Abderrahmane and BRAHIM Hani. Supervised by: Ms. S. Bouraine and Ms. Dj. Naceur. Affiliation: Université SAAD DAHLAB de BLIDA, Faculté de Technologie, Département d'Électronique, Algeria, Academic Year 2024-2025.

