
# ROS2 Keyboard to Joy

`keyboard_joy` is a ROS2 package that allows you to simulate joystick input using your keyboard. This is particularly useful for testing and development when a physical joystick is not available.

Tested on ROS2 Humble.

## Installation

1. **Install ROS2**: Follow the official ROS2 installation guide for your operating system.

2. **Install the `pynput` Library**:

   This package requires `pynput` to capture keyboard input. Install it using pip:

   ```bash
   pip3 install pynput
   ```

3. **Clone the `keyboard_joy` Package**:

   Navigate to your ROS2 workspace's `src` directory and clone the package repository:

   ```bash
   cd ~/your_ros2_workspace/src
   git clone https://github.com/atarbabgei/keyboard_joy.git
   ```

4. **Build the Package**:

   Build your workspace to compile the package:

   ```bash
   cd ~/your_ros2_workspace
   colcon build --packages-select keyboard_joy
   ```

5. **Source the Workspace**:

   Source your workspace to overlay your environment:

   ```bash
   source install/setup.bash
   ```

## Usage

To run the `keyboard_joy` node using the default configuration (as stored in `/config/key_mappings.yaml`), execute the following command:

```bash
ros2 launch keyboard_joy keyboard_joy.launch.py
```

However, if you want to use a custom YAML configuration file for your project, run the following command:

```bash
ros2 launch keyboard_joy keyboard_joy.launch.py config:=/your_path/your_config_file.yaml
```

Replace `/your_path/your_config_file.yaml` with the path to your custom YAML file.

## Configuration

The key mappings for the joystick simulation are configured using a YAML file which allows you to customize which keyboard keys map to specific joystick axes and buttons.

### Example YAML Configuration

```yaml
# Config for mapping keyboard keys to joystick axes and buttons

axes:
  # Axis mappings: Define which keyboard keys control specific joystick axes
  # Format: '<key>: [<axis_index>, <axis_value>, <mode>'
  w: [0, 1.0, 'normal'] 
  s: [0, -1.0, 'normal'] 
  a: [1, 1.0, 'normal']
  d: [1, -1.0, 'normal']
  Key.up: [2, 1.0, 'sticky']   
  Key.down: [2, -1.0, 'sticky']
  Key.left: [3, 1.0, 'sticky']
  Key.right: [3, -1.0, 'sticky']

# Parameters for axis increment_rate, increment_step
parameters:
  axis_increment_rate: 0.01   # Time interval for updating axis values
  axis_increment_step: 0.01   # Step size for axis value change

buttons:
  # Button mappings: Define which keyboard keys toggle specific joystick buttons
  # Format: '<key>: <button_index>'
  '0': 0 
  '1': 1 
  '2': 2 
  '3': 3 
```