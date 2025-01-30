
# Drone Balancing and Hand Recognition System

## Overview

This project integrates machine learning and computer vision to create a drone system capable of self-balancing and being controlled remotely through hand gestures. The main goal is to combine drone control with hand recognition to provide an intuitive and interactive user experience, allowing users to drive the drone with hand gestures, and maintain stable flight through AI-powered balancing systems.

## Features

- **Drone Balancing**: Utilizes machine learning algorithms to maintain stable flight by adjusting the drone's movements in real-time.
- **Hand Gesture Recognition**: Leverages computer vision techniques to recognize hand gestures that control the drone’s direction and speed.
- **Remote Control**: Allows for controlling the drone remotely through simple hand gestures, improving accessibility and user interaction.
- **Real-Time Control**: Real-time feedback and control loop to ensure the drone maintains stability while responding to user input.

## Project Components

1. **Computer Vision**:
   - Implemented using OpenCV and MediaPipe to detect hand gestures.
   - Gesture recognition translates user input into commands for the drone.
  
2. **Drone Control**:
   - Integrated with drone hardware to maintain stability, using balancing algorithms based on sensor input.
   - Balancing algorithms ensure stable flight, preventing the drone from tipping or losing control.

3. **Machine Learning**:
   - AI models trained to process sensor data and adjust the drone's orientation in real-time.
   - Hand gestures are mapped to specific control commands using deep learning techniques.

## Getting Started

### Prerequisites

To run this project, you’ll need the following installed:

```bash
# Install Python 3.x
sudo apt install python3

# Install required Python libraries
pip install opencv-python
pip install mediapipe
pip install tensorflow

# Other dependencies listed in 'requirements.txt'
```
### Installation

1. Clone the repository:

```bash
git clone https://github.com/IvanDaGomez/Drone.git
cd Drone
```

2. Install the necessary dependencies:

```bash
pip install -r requirements.txt
```

3. If you are working with drone hardware, make sure to follow the specific setup instructions for your drone model. If using a simulation, refer to the simulation setup documentation.

### Running the Project

1. **Balancing Control**: 

   The balancing algorithm uses sensors (e.g., gyroscopes, accelerometers) to maintain stable flight. Connect the drone hardware and ensure the sensor data is streaming to the system.

```bash
# Example of running the balancing control script (if applicable):
python drone_balancing.py
```

2. **Hand Gesture Recognition**: 

   Run the hand gesture recognition script:

```bash
python hand_gesture_control.py
```

   This script will use your webcam to detect hand gestures and send control signals to the drone. The hand gestures are mapped to various drone movements (e.g., moving forward, backward, left, right, up, down).

3. **Test Flight**: 

   For safety, ensure you are in a controlled environment with enough space for the drone to fly and respond to commands.

### Usage

- **Forward Gesture**: Move your hand forward to make the drone move forward.
- **Backward Gesture**: Move your hand back to make the drone move backward.
- **Left/Right Gestures**: Move your hand left or right to steer the drone in that direction.
- **Up/Down Gestures**: Raise or lower your hand to control altitude.
  
  You can adjust these gestures in the code to match your preferred hand movements or control scheme.

### Troubleshooting

- **Drone not responding to gestures**: Ensure the webcam is working properly and that MediaPipe is detecting your hand gestures.
- **Balancing issues**: Make sure the drone's sensors are correctly calibrated and that the control algorithm is receiving the right input.

## Acknowledgments

- **OpenCV**: For powerful image processing capabilities to detect hand gestures.
- **MediaPipe**: For fast and accurate hand tracking.
- **TensorFlow**: For machine learning models used in gesture recognition.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
