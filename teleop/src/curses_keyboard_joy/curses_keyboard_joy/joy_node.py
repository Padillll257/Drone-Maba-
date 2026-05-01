#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import threading
import yaml
import os
import curses
import time
from ament_index_python.packages import get_package_share_directory

class CursesKeyboardJoy(Node):
    def __init__(self, stdscr):
        super().__init__('curses_keyboard_joy')
        self.stdscr = stdscr

        # Configure curses
        if self.stdscr:
            curses.curs_set(0)  # Hide cursor
            self.stdscr.nodelay(True)  # Non-blocking getch
            self.stdscr.clear()
            self.stdscr.addstr(0, 0, "Curses Keyboard Joy Node Started. Press 'q' to quit.")
            self.stdscr.refresh()

        self.declare_parameter('config', '')
        self.load_key_mappings()

        self.get_logger().info("CursesKeyboardJoy Node Started")
        
        self.joy_publisher = self.create_publisher(Joy, 'joy', 10)

        max_axis_index = max([v[0] for v in self.axis_mappings.values()]) if self.axis_mappings else 0
        max_button_index = max(self.button_mappings.values()) if self.button_mappings else 0
        
        self.joy_msg = Joy()
        self.joy_msg.axes = [0.0] * (max_axis_index + 1)
        self.joy_msg.buttons = [0] * (max_button_index + 1)

        self.active_axes = {}
        self.sticky_axes = {}
        
        # To handle key release, we track the last time a key was pressed
        self.key_last_pressed = {}
        self.key_release_timeout = 0.2  # Seconds without press to consider released

        self.lock = threading.Lock()

        self.timer = self.create_timer(0.1, self.publish_joy)
        self.increment_timer = self.create_timer(self.axis_increment_rate, self.update_active_axes)

        # Thread for ROS spinning so curses can block/run in main thread
        self.ros_thread = threading.Thread(target=rclpy.spin, args=(self,), daemon=True)
        self.ros_thread.start()

    def load_key_mappings(self):
        """Load key mappings and parameters from a YAML file."""
        config_file_path = self.get_parameter('config').get_parameter_value().string_value

        if not config_file_path:
            # Note: During initial build, the package share directory might not be fully linked,
            # so we fallback to a relative path for local testing if needed, though get_package_share is standard.
            try:
                config_file_path = os.path.join(get_package_share_directory('curses_keyboard_joy'), 'config', 'key_mappings.yaml')
            except Exception:
                config_file_path = "config/key_mappings.yaml"

        try:
            with open(config_file_path, 'r') as file:
                key_mappings = yaml.safe_load(file)
        except Exception as e:
            key_mappings = {}

        raw_axis_mappings = key_mappings.get('axes', {})
        raw_button_mappings = key_mappings.get('buttons', {})
        parameters = key_mappings.get('parameters', {})

        self.axis_increment_rate = parameters.get('axis_increment_rate', 0.1)
        self.axis_increment_step = parameters.get('axis_increment_step', 0.05)

        # Convert string mappings to characters/keys
        self.axis_mappings = self._convert_map_keys(raw_axis_mappings)
        self.button_mappings = self._convert_map_keys(raw_button_mappings)

    def _convert_map_keys(self, mapping):
        """Convert YAML key strings (e.g., 'w', 'Key.up') to integer key codes or strings suitable for curses."""
        converted = {}
        for k, v in mapping.items():
            if k == 'Key.up':
                converted[curses.KEY_UP] = v
            elif k == 'Key.down':
                converted[curses.KEY_DOWN] = v
            elif k == 'Key.left':
                converted[curses.KEY_LEFT] = v
            elif k == 'Key.right':
                converted[curses.KEY_RIGHT] = v
            elif len(k) == 1:
                converted[ord(k.lower())] = v  # Handle lowercase
                converted[ord(k.upper())] = v  # Handle uppercase
            else:
                converted[k] = v
        return converted

    def process_keys(self):
        """Main curses loop run in the foreground."""
        try:
            while rclpy.ok():
                if not self.stdscr:
                    time.sleep(0.1)
                    continue

                key = self.stdscr.getch()
                now = time.time()

                with self.lock:
                    if key == ord('q'):
                        break

                    if key != curses.ERR:
                        self.key_last_pressed[key] = now
                        self._handle_keypress(key)

                    self._check_key_releases(now)
                
                # Small sleep to prevent burning CPU
                time.sleep(0.01)
        except KeyboardInterrupt:
            pass

    def _handle_keypress(self, key):
        if key in self.axis_mappings:
            axis, value, mode = self.axis_mappings[key]
            if mode == 'sticky':
                self.sticky_axes[axis] = self.sticky_axes.get(axis, 0.0) + value * self.axis_increment_step
                self.joy_msg.axes[axis] = round(max(min(self.sticky_axes[axis], 1.0), -1.0), 4)
            else:
                self.active_axes[axis] = value
        elif key in self.button_mappings:
            button_index = self.button_mappings[key]
            self.joy_msg.buttons[button_index] = 1

    def _check_key_releases(self, now):
        """Simulate key release if a key hasn't been pressed recently."""
        released_keys = []
        for key, last_time in self.key_last_pressed.items():
            if now - last_time > self.key_release_timeout:
                released_keys.append(key)
        
        for key in released_keys:
            self._handle_keyrelease(key)
            del self.key_last_pressed[key]

    def _handle_keyrelease(self, key):
        if key in self.axis_mappings:
            axis, _, mode = self.axis_mappings[key]
            if axis in self.active_axes:
                del self.active_axes[axis]
            if mode != 'sticky':
                self.joy_msg.axes[axis] = 0.0
        elif key in self.button_mappings:
            button_index = self.button_mappings[key]
            self.joy_msg.buttons[button_index] = 0

    def publish_joy(self):
        with self.lock:
            self.joy_msg.header.stamp = self.get_clock().now().to_msg()
            self.joy_publisher.publish(self.joy_msg)
            
            # Update curses display with current joy state
            if self.stdscr:
                self.stdscr.move(2, 0)
                self.stdscr.clrtoeol()
                self.stdscr.addstr(2, 0, f"Axes: {[round(a, 2) for a in self.joy_msg.axes]}")
                self.stdscr.move(3, 0)
                self.stdscr.clrtoeol()
                self.stdscr.addstr(3, 0, f"Buttons: {self.joy_msg.buttons}")
                self.stdscr.refresh()

    def update_active_axes(self):
        with self.lock:
            for axis, target_value in list(self.active_axes.items()):
                current_value = self.joy_msg.axes[axis]
                if target_value > 0:
                    self.joy_msg.axes[axis] = round(min(current_value + self.axis_increment_step, target_value), 4)
                else:
                    self.joy_msg.axes[axis] = round(max(current_value - self.axis_increment_step, target_value), 4)

def run_curses_node(stdscr):
    # Initializes ROS context inside curses wrapper
    rclpy.init()
    node = CursesKeyboardJoy(stdscr)
    try:
        node.process_keys()
    finally:
        node.destroy_node()
        rclpy.shutdown()

def main(args=None):
    # We use curses wrapper to handle terminal state gracefully
    curses.wrapper(run_curses_node)

if __name__ == '__main__':
    main()
