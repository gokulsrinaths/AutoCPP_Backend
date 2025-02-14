# AutoCPP_Backend

Backend system for the Cal Poly Pomona Autonomous Vehicle project. This system handles GPS navigation, object detection, and autonomous vehicle control.

## Features

- Real-time object detection using YOLO
- GPS-based navigation
- Path planning and obstacle avoidance
- Vehicle state management
- API integration for ride booking
- Simulation capabilities for testing

## Requirements

- ROS2 Humble
- Python 3.8+
- OpenCV
- PyTorch (for YOLO)
- GPS module
- USB camera

## Installation

1. Clone the repository:
```bash
git clone https://github.com/your-username/AutoCPP_Backend.git
cd AutoCPP_Backend
```

2. Install dependencies:
```bash
pip install -r requirements.txt
```

3. Build the ROS2 package:
```bash
colcon build
source install/setup.bash  # or setup.ps1 on Windows
```

## Usage

1. Start the system:
```bash
ros2 launch AutoCPP_Backend autonomous_system.launch.py
```

2. For simulation:
```bash
ros2 launch AutoCPP_Backend simulation.launch.py
```

## Configuration

- GPS settings: `config/gps_config.yaml`
- Camera settings: `config/camera_config.yaml`
- Navigation parameters: `config/navigation_config.yaml`
- Vehicle parameters: `config/vehicle_config.yaml`

## Directory Structure

```
AutoCPP_Backend/
├── perception/        # Object detection
├── navigation/        # Path planning
├── hardware_interface/# GPS interface
├── control/          # Vehicle control
├── state/           # State management
├── config/          # Configuration files
├── launch/          # Launch files
└── simulation/      # Simulation tools
```

## Contributing

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

## License

This project is licensed under the MIT License - see the LICENSE file for details. 