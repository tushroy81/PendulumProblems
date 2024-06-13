# Inverted Pendulum
## How-to
1. Clone this repository in /src folder of your ros2 workspace
2. Build and source the workspace. Navigate to your workspace directory and run
```
colcon build --symlink-install
source install/setup.bash
```
3. You can now launch the simulation

**Single Inverted Pendulum - Balance**
```
ros2 launch single_inverted single_inverted_pendulum.launch.py
```
   **Single Inverted Pendulum - Swing-up**
```
ros2 launch single_inverted single_inverted_pendulum2.launch.py
```
   **Double Inverted Pendulum**
```
ros2 launch double_pendulum double_inverted_pendulum.launch.py
```
