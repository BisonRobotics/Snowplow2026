#!/usr/bin/env python3
"""ROS 2 node for articulated steering via a Roboteq SDC2160 (ASCII serial protocol).

Subscribes:  steering_angle_cmd  (std_msgs/Float64)      target angle [rad]
Publishes:   steering_state      (sensor_msgs/JointState) position [rad], effort [A]

The PA-04-xxx-HS actuator's Hall sensors feed the SDC2160 Encoder 1 input, and the
channel runs in Closed Loop Count Position (^MMOD = 3) with PID gains tuned.

Hall counts are RELATIVE, so on startup the node homes: it switches the channel to
open loop, retracts at low power until the count stops changing (the actuator's
internal limit switch cuts the motor), zeroes the counter (!C), and returns to
position mode. A positive motor command must extend the actuator (set Motor
Direction in Roborun) and extending must count upward (swap A/B or use `invert`).
"""
import time

import rclpy
import serial
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

FAULT_NAMES = [
    'overheat', 'overvoltage', 'undervoltage', 'short_circuit',
    'emergency_stop', 'sensor_fault', 'mosfet_failure', 'default_config_loaded',
]


class SteeringNode(Node):
    def __init__(self):
        super().__init__('steering_controller')

        self.declare_parameter('port', '/dev/ttyACM2')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('serial_timeout_s', 0.1)
        self.declare_parameter('channel', 1)
        # CALIBRATE: counts are measured from the retracted end after homing.
        self.declare_parameter('counts_per_rad', 100000.0) # encoder counts per radian
        self.declare_parameter('center_counts', 300000)    # counts at straight-ahead
        self.declare_parameter('travel_counts', 0)         # full stroke in counts; >0 clamps to
                                                           # [home_counts, home_counts+travel]
        self.declare_parameter('home_on_start', True)
        self.declare_parameter('home_counts', 0)           # counter value at the retracted end
        self.declare_parameter('home_power_permil', -300)  # open-loop command used to retract
        self.declare_parameter('home_stall_s', 0.8)        # no count change this long = at stop
        self.declare_parameter('home_timeout_s', 45.0)
        self.declare_parameter('reconnect_tolerance_counts', 5000)
        self.declare_parameter('invert', False)
        self.declare_parameter('max_angle_rad', 0.6)       # software limit, +/-
        self.declare_parameter('update_rate_hz', 20.0)
        self.declare_parameter('cmd_timeout_s', 0.5)       # hold position if no cmd within this
        self.declare_parameter('controller_watchdog_ms', 1000)  # ^RWD; 0 disables
        self.declare_parameter('joint_name', 'steering_joint')

        p = self.get_parameter
        self._port = p('port').value
        self._baud = p('baudrate').value
        self._ser_timeout = p('serial_timeout_s').value
        self._ch = p('channel').value
        self._cpr = p('counts_per_rad').value
        self._center = p('center_counts').value
        self._sign = -1.0 if p('invert').value else 1.0
        self._max_angle = p('max_angle_rad').value
        self._cmd_timeout = p('cmd_timeout_s').value
        self._rwd_ms = p('controller_watchdog_ms').value
        self._joint = p('joint_name').value
        self._travel = int(p('travel_counts').value)
        self._home_enabled = p('home_on_start').value
        self._home_counts = int(p('home_counts').value)
        self._home_power = int(p('home_power_permil').value)
        self._home_stall = p('home_stall_s').value
        self._home_timeout = p('home_timeout_s').value
        self._reconnect_tol = int(p('reconnect_tolerance_counts').value)

        self._ser = None
        self._target_rad = None
        self._last_cmd_time = self.get_clock().now()
        self._last_faults = 0
        self._stale_warned = False
        self._pos_counts = None
        self._homed = False
        self._home_state = 'start'
        self._home_t0 = 0.0
        self._home_last_change = 0.0
        self._home_last_count = 0

        self.create_subscription(Float64, 'steering_angle_cmd', self._on_cmd, 10)
        self._state_pub = self.create_publisher(JointState, 'steering_state', 10)
        self.create_timer(1.0 / p('update_rate_hz').value, self._update)

    # ------------------------------------------------------------ conversions
    def _rad_to_counts(self, rad):
        counts = int(round(self._center + self._sign * rad * self._cpr))
        if self._travel > 0:
            counts = max(self._home_counts, min(self._home_counts + self._travel, counts))
        return counts

    def _counts_to_rad(self, counts):
        return self._sign * (counts - self._center) / self._cpr

    # ---------------------------------------------------------------- serial
    def _connect(self):
        try:
            self._ser = serial.Serial(self._port, self._baud, timeout=self._ser_timeout)
            self._send('^ECHOF 1')                      # disable command echo
            if self._rwd_ms > 0:
                self._send(f'^RWD {int(self._rwd_ms)}')  # controller-side serial watchdog
            if self._homed:
                # The counter survives a serial glitch but not a controller power cycle.
                counts = self._query('C', self._ch)
                if self._pos_counts is not None and \
                        abs(counts - self._pos_counts) > self._reconnect_tol:
                    self.get_logger().warning('Encoder count jumped; controller reset? re-homing')
                    self._homed = False
                    self._home_state = 'start'
                else:
                    self._send(f'!P {self._ch} {counts}')   # hold where we are
            self.get_logger().info(f'Connected to SDC2160 on {self._port}')
            return True
        except (serial.SerialException, OSError, TimeoutError) as e:
            self.get_logger().warning(f'Connect failed: {e}', throttle_duration_sec=5.0)
            self._close()
            return False

    def _close(self):
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
        self._ser = None

    def _transact(self, cmd):
        """Send a command, return the first non-empty, non-echo reply line."""
        self._ser.reset_input_buffer()
        self._ser.write((cmd + '\r').encode('ascii'))
        deadline = time.monotonic() + 3 * self._ser_timeout
        while time.monotonic() < deadline:
            raw = self._ser.read_until(b'\r')
            if not raw.endswith(b'\r'):
                break
            line = raw.decode('ascii', 'ignore').strip()
            if line and line != cmd:
                return line
        raise TimeoutError(f'No reply to "{cmd}"')

    def _send(self, cmd):
        reply = self._transact(cmd)
        if not reply.startswith('+'):
            raise IOError(f'Command "{cmd}" rejected: {reply}')

    def _query(self, name, channel=None):
        cmd = f'?{name}' + (f' {channel}' if channel is not None else '')
        reply = self._transact(cmd)
        prefix = f'{name}='
        if not reply.startswith(prefix):
            raise IOError(f'Unexpected reply to "{cmd}": {reply}')
        return int(reply[len(prefix):])

    # ------------------------------------------------------------- callbacks
    def _on_cmd(self, msg):
        angle = max(-self._max_angle, min(self._max_angle, msg.data))
        if angle != msg.data:
            self.get_logger().warning(
                f'Command {msg.data:.3f} rad clamped to {angle:.3f}', throttle_duration_sec=2.0)
        self._target_rad = angle
        self._last_cmd_time = self.get_clock().now()
        self._stale_warned = False

    def _update(self):
        if self._ser is None and not self._connect():
            return
        try:
            if not self._homed:
                self._home_step()
            else:
                self._read_feedback()
                self._write_command()
        except (serial.SerialException, OSError, TimeoutError, IOError, ValueError) as e:
            self.get_logger().error(f'Controller communication error: {e}',
                                    throttle_duration_sec=2.0)
            self._abort_homing()
            self._close()

    # ---------------------------------------------------------------- homing
    def _home_step(self):
        ch, now = self._ch, time.monotonic()
        if self._home_state == 'start':
            if not self._home_enabled:
                counts = self._query('C', ch)
                self._send(f'!P {ch} {counts}')
                self._pos_counts = counts
                self._homed = True
                self.get_logger().warning('Homing disabled: trusting existing encoder count')
                return
            self._send(f'^MMOD {ch} 0')                    # open loop for homing
            self._home_last_count = self._query('C', ch)
            self._home_t0 = self._home_last_change = now
            self._send(f'!G {ch} {self._home_power}')
            self._home_state = 'moving'
            self.get_logger().info('Homing: retracting to the limit switch')
            return

        counts = self._query('C', ch)
        if abs(counts - self._home_last_count) > 3:
            self._home_last_count = counts
            self._home_last_change = now
        if now - self._home_last_change > self._home_stall:
            self._send(f'!G {ch} 0')
            self._send(f'!C {ch} {self._home_counts}')       # zero the counter here
            self._send(f'!P {ch} {self._home_counts}')       # target = here, before re-enabling
            self._send(f'^MMOD {ch} 3')                      # back to count position
            self._send(f'!P {ch} {self._home_counts}')
            self._pos_counts = self._home_counts
            self._homed = True
            self._home_state = 'start'
            self.get_logger().info(f'Homing complete; counter set to {self._home_counts}')
        elif now - self._home_t0 > self._home_timeout:
            raise TimeoutError('Homing timed out (no limit stop detected)')
        else:
            self._send(f'!G {ch} {self._home_power}')        # keep the watchdog fed

    def _abort_homing(self):
        if self._home_state == 'moving' and self._ser is not None:
            try:
                self._send(f'!G {self._ch} 0')
            except Exception:
                pass
        self._home_state = 'start'

    def _read_feedback(self):
        self._pos_counts = self._query('C', self._ch)          # Hall/encoder count, absolute
        amps = self._query('A', self._ch) / 10.0               # motor amps are reported x10
        faults = self._query('FF')

        if faults != self._last_faults:
            active = [n for i, n in enumerate(FAULT_NAMES) if faults & (1 << i)]
            self.get_logger().warning(f'Fault flags changed: {active or "none"}')
            self._last_faults = faults

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = [self._joint]
        js.position = [self._counts_to_rad(self._pos_counts)]
        js.effort = [amps]
        self._state_pub.publish(js)

    def _write_command(self):
        if self._target_rad is None:
            return  # never move before the first command arrives

        age = (self.get_clock().now() - self._last_cmd_time).nanoseconds * 1e-9
        if age > self._cmd_timeout:
            # Stale command: hold wherever we currently are.
            if not self._stale_warned:
                self.get_logger().warning('Steering command timed out; holding position')
                self._stale_warned = True
            target_counts = self._pos_counts
        else:
            target_counts = self._rad_to_counts(self._target_rad)

        # Re-sent every cycle, which also keeps the controller watchdog fed.
        self._send(f'!P {self._ch} {target_counts}')

    # -------------------------------------------------------------- shutdown
    def destroy_node(self):
        if self._ser is not None:
            try:
                self._send(f'!MS {self._ch}')  # stop motor
            except Exception:
                pass
            self._close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SteeringNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
