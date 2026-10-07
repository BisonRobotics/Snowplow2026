"""
SDC2160 motor controller driver for ROS 2 (Python / rclpy)

What this node does
-------------------
1. Listens for velocity commands on the /cmd_vel topic (geometry_msgs/Twist).
2. Converts them into separate left and right wheel power values
   (this is called "differential drive kinematics").
3. Sends those values to the Roboteq SDC2160 over a serial port using
   Roboteq's ASCII command protocol.
4. Stops the motors automatically if commands stop arriving (safety watchdog).
5. Publishes the battery voltage on /battery_voltage.

Roboteq protocol basics (verify against the SDC2160 manual):
  - Commands are plain text ending with a carriage return (\\r).
  - "!G 1 500"  -> set motor channel 1 to 50.0% power (range -1000..1000)
  - "?V 2"      -> query the battery voltage; reply looks like "V=125" (12.5 V)
  - "^ECHOF 1"  -> turn off the controller echoing back what we send it
  - The controller answers commands with "+" (ok) or "-" (error).
"""

import threading  # used for a lock so two callbacks can't use the serial port at once

import serial  # pyserial: lets us talk to /dev/ttyACMx like a file

import rclpy  # the ROS 2 Python client library
from rclpy.node import Node  # base class every ROS 2 node inherits from
from geometry_msgs.msg import Twist  # message type: linear + angular velocity
from std_msgs.msg import Float32  # message type: a single floating point number


class SDC2160Node(Node):
    """A ROS 2 node that turns /cmd_vel messages into SDC2160 motor commands."""

    def __init__(self):
        # Give the node its name. This is what shows up in `ros2 node list`.
        super().__init__('sdc2160_driver')

        # ------------------------------------------------------------------
        # Parameters
        # ------------------------------------------------------------------
        # Parameters are settings you can change at launch time without
        # editing code, e.g.:
        #   ros2 run sdc2160_driver driver_node --ros-args -p port:=/dev/roboteq
        # Each declare_parameter call gives the name and a default value.
        self.declare_parameter('port', '/dev/ttyACM0')      # serial device path
        self.declare_parameter('baud', 115200)              # SDC2160 USB default; check yours
        self.declare_parameter('wheel_separation', 0.30)    # meters between left and right wheels
        self.declare_parameter('max_linear_speed', 1.0)     # m/s that should equal 100% motor power
        self.declare_parameter('invert_left', False)        # flip left motor direction
        self.declare_parameter('invert_right', True)        # flip right motor (motors usually face opposite ways)
        self.declare_parameter('cmd_timeout', 0.5)          # seconds of silence before we stop the motors

        # Read the parameter values into plain Python variables.
        # (`p` is just a short alias so the lines below stay readable.)
        p = self.get_parameter
        self.sep = p('wheel_separation').value
        self.vmax = p('max_linear_speed').value
        self.timeout = p('cmd_timeout').value

        # Store direction inversion as a multiplier: -1 flips the motor, 1 leaves it.
        self.inv_l = -1 if p('invert_left').value else 1
        self.inv_r = -1 if p('invert_right').value else 1

        # ------------------------------------------------------------------
        # Serial connection to the controller
        # ------------------------------------------------------------------
        # timeout=0.05 means a read gives up after 50 ms instead of blocking
        # forever, so a missing reply can't freeze the node.
        self.ser = serial.Serial(p('port').value, p('baud').value, timeout=0.05)

        # ROS can run several callbacks on different threads. The lock makes
        # sure only one of them talks on the serial port at a time, otherwise
        # two commands could get mixed together into garbage.
        self.lock = threading.Lock()

        # Tell the controller not to echo back every character we send.
        # This keeps replies short and easy to parse.
        self._send('^ECHOF 1')

        # ------------------------------------------------------------------
        # ROS interfaces (topics)
        # ------------------------------------------------------------------
        # Subscribe to 'cmd_vel'. Every time a Twist message arrives,
        # self.cmd_cb is called. The 10 is the queue depth (how many messages
        # to buffer if we fall behind).
        self.create_subscription(Twist, 'cmd_vel', self.cmd_cb, 10)

        # Publisher for battery voltage so other nodes (or you, using
        # `ros2 topic echo /battery_voltage`) can monitor it.
        self.batt_pub = self.create_publisher(Float32, 'battery_voltage', 10)

        # Remember when we last received a command; the watchdog uses this.
        self.last_cmd = self.get_clock().now()

        # ------------------------------------------------------------------
        # Timers
        # ------------------------------------------------------------------
        # Call watchdog_cb every 0.1 s to check whether commands went quiet.
        self.create_timer(0.1, self.watchdog_cb)
        # Call battery_cb once per second to read and publish the voltage.
        self.create_timer(1.0, self.battery_cb)

        self.get_logger().info('SDC2160 driver started')

    # ======================================================================
    # Low-level serial helpers
    # ======================================================================
    def _send(self, cmd):
        """Send a command and ignore the reply (used for motor commands)."""
        with self.lock:  # only one thread may use the port at a time
            # Roboteq commands end with a carriage return, not a newline.
            self.ser.write((cmd + '\r').encode())
            # The controller replies with "+" for success. We don't need it,
            # so clear it out so it doesn't pile up and confuse later queries.
            self.ser.reset_input_buffer()

    def _query(self, cmd):
        """Send a query (starts with '?') and return the reply as a string."""
        with self.lock:
            # Clear any leftover bytes first so we read only the new reply.
            self.ser.reset_input_buffer()
            self.ser.write((cmd + '\r').encode())
            # Read until the carriage return that ends the reply line.
            return self.ser.read_until(b'\r').decode(errors='ignore').strip()

    def set_motors(self, left, right):
        """
        Set both motors.

        left and right are fractions from -1.0 (full reverse) to 1.0
        (full forward). The Roboteq expects integers from -1000 to 1000,
        so we scale by 1000.
        """
        # Apply the direction inversion, clamp to the valid range so a big
        # command can never exceed 100%, then scale to the Roboteq range.
        l = int(max(-1.0, min(1.0, left * self.inv_l)) * 1000)
        r = int(max(-1.0, min(1.0, right * self.inv_r)) * 1000)

        # "!G <channel> <value>" is Roboteq's "Go" command: set motor power.
        self._send(f'!G 1 {l}')  # channel 1 = left motor
        self._send(f'!G 2 {r}')  # channel 2 = right motor

    # ======================================================================
    # ROS callbacks
    # ======================================================================
    def cmd_cb(self, msg: Twist):
        """Runs every time a /cmd_vel message arrives."""
        # A Twist has linear and angular velocity in 3D. A ground robot only
        # uses two of those values:
        v = msg.linear.x    # forward speed in m/s (negative = backward)
        w = msg.angular.z   # turning rate in rad/s (positive = counter-clockwise)

        # Differential drive math: to turn left, the right wheel must go
        # faster than the left. Each wheel's speed is the forward speed
        # plus/minus half the turning contribution.
        #   left  = v - w * (wheel_separation / 2)
        #   right = v + w * (wheel_separation / 2)
        # Dividing by vmax converts m/s into a -1.0..1.0 power fraction.
        left = (v - w * self.sep / 2.0) / self.vmax
        right = (v + w * self.sep / 2.0) / self.vmax

        self.set_motors(left, right)

        # Note the time so the watchdog knows we're still getting commands.
        self.last_cmd = self.get_clock().now()

    def watchdog_cb(self):
        """Safety: stop the motors if no command has arrived recently."""
        # Time since the last command, converted from nanoseconds to seconds.
        age = (self.get_clock().now() - self.last_cmd).nanoseconds / 1e9
        if age > self.timeout:
            # Without this, the robot would keep driving at its last
            # commanded speed if the teleop or navigation node crashed.
            self.set_motors(0.0, 0.0)

    def battery_cb(self):
        """Read the battery voltage from the controller and publish it."""
        # "?V 2" asks for the main battery voltage. The reply looks like
        # "V=125", meaning 12.5 V (the value is in tenths of a volt).
        reply = self._query('?V 2')
        try:
            # Split "V=125" at the "=", take "125", convert to volts.
            volts = int(reply.split('=')[1]) / 10.0
            self.batt_pub.publish(Float32(data=volts))
        except (IndexError, ValueError):
            # The reply was empty or garbled (e.g. a timeout). Skip this
            # reading rather than crashing the whole node.
            pass

    # ======================================================================
    # Shutdown
    # ======================================================================
    def destroy_node(self):
        """Called when the node shuts down (e.g. you press Ctrl+C)."""
        try:
            # Always stop the motors and release the port on the way out.
            self.set_motors(0.0, 0.0)
            self.ser.close()
        finally:
            # `finally` guarantees the normal ROS cleanup still runs even if
            # something above raised an error.
            super().destroy_node()


def main():
    """Entry point. setup.py points the `driver_node` command at this function."""
    rclpy.init()                 # start the ROS 2 client library
    node = SDC2160Node()         # create our node (opens the serial port)
    try:
        rclpy.spin(node)         # keep running and process callbacks until stopped
    except KeyboardInterrupt:
        pass                     # Ctrl+C is a normal way to quit, not an error
    finally:
        node.destroy_node()      # stop motors and close the port
        rclpy.shutdown()         # shut down ROS 2 cleanly


if __name__ == '__main__':
    main()