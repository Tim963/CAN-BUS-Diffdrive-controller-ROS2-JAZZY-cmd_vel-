import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import can
from rclpy.duration import Duration

# === Plattformparameter ===
WHEEL_DISTANCE = 1.25        # [m] Kettenabstand
SPEED_SCALE = 850.0          # [CAN/m/s]
MAX_CAN_SPEED = 2550         # max. CAN-Wert laut Arduino
RAMP_STEP = 100              # max. Änderung pro 0.1s kleiner weicher 255 entspricht keiner wirkung
TIMEOUT_SEC = 0.5            # Timeout-Schwelle für cmd_vel

# === Sicherheitsgrenzen für cmd_vel ===
MAX_V = 3.0                  # [m/s] maximal lineare Geschwindigkeit #0.22            # [m/s] linear
MAX_OMEGA = 2.84             # [rad/s] maximale Winkelgeschwindigkeit im Stand #max möglich (bei Spurbreite: 1.25 und unter annahme dass bei maxcanspeed 2550 3m/s lin geschw.): 4.8    # [rad/s] angular

OMEGA_AT_VMAX = 0.3          # [rad/s] bei voller Fahrt (v = MAX_V)

# === CAN-Bus Setup ===
can_interface = 'can0'
bus = can.Bus(interface='socketcan', channel=can_interface)

def clamp(val, min_val, max_val, label=None, node=None):
    clamped = max(min(val, max_val), min_val)
    if clamped != val and node:
        node.get_logger().warn(
            f"[clamp] {label or 'Wert'} wurde begrenzt: {val} → {clamped}"
        )
    return clamped

def send_speed(speed_l, speed_r):
    def to_bytes(val):
        val = int(val) & 0xFFFF
        return [(val >> 8) & 0xFF, val & 0xFF]

    l_msb, l_lsb = to_bytes(speed_l)
    r_msb, r_lsb = to_bytes(speed_r)

    msg_l = [0x2B, 0x00, 0x20, 0x02, l_msb, l_lsb, 0x10, 0x00]
    msg_r = [0x2B, 0x00, 0x20, 0x02, r_msb, r_lsb, 0x10, 0x00]

    bus.send(can.Message(arbitration_id=0x601, data=msg_l, is_extended_id=False))
    bus.send(can.Message(arbitration_id=0x602, data=msg_r, is_extended_id=False))

    print(f"[→] L: {speed_l}   R: {speed_r}")

class DiffDriveController(Node):
    def __init__(self):
        super().__init__('diffdrive_controller')

        self.subscription = self.create_subscription(TwistStamped, '/cmd_vel', self.listener_callback, 10)
        self.create_timer(0.1, self.update_loop)

        self.last_cmd_time = self.get_clock().now()
        self.timeout_triggered = False

        self.target_speed_l = 0
        self.target_speed_r = 0
        self.actual_speed_l = 0
        self.actual_speed_r = 0

        self.get_logger().info('Node gestartet: /cmd_vel → CAN mit Rampen, Timeout und dynamischem ω-Limit')

    def listener_callback(self, msg: TwistStamped):
        v_raw = -msg.twist.linear.x
        omega_raw = -msg.twist.angular.z

        v = clamp(v_raw, -MAX_V, MAX_V, label='linear.x (m/s)', node=self)

        def dynamic_omega_limit(v):
            ratio = abs(v) / MAX_V
            return MAX_OMEGA * (1 - ratio) + OMEGA_AT_VMAX * ratio

        omega_limit = dynamic_omega_limit(v)
        omega = clamp(omega_raw, -omega_limit, omega_limit, label='angular.z (dynamisch)', node=self)

        v_l = v - (omega * WHEEL_DISTANCE / 2)
        v_r = v + (omega * WHEEL_DISTANCE / 2)

        raw_l = int(v_l * SPEED_SCALE)
        raw_r = int(v_r * SPEED_SCALE)

        self.target_speed_l = clamp(raw_l, -MAX_CAN_SPEED, MAX_CAN_SPEED, label='CAN speed L', node=self)
        self.target_speed_r = clamp(raw_r, -MAX_CAN_SPEED, MAX_CAN_SPEED, label='CAN speed R', node=self)

        self.last_cmd_time = self.get_clock().now()
        self.timeout_triggered = False

    def update_loop(self):
        now = self.get_clock().now()
        elapsed = now - self.last_cmd_time

        if elapsed > Duration(seconds=TIMEOUT_SEC):
            if not self.timeout_triggered:
                self.get_logger().warn(f"[Timeout] Kein cmd_vel seit {elapsed.nanoseconds / 1e9:.2f}s → Stoppe Motoren")
            self.target_speed_l = 0
            self.target_speed_r = 0
            self.timeout_triggered = True

        self.actual_speed_l = self.apply_ramp(self.actual_speed_l, self.target_speed_l)
        self.actual_speed_r = self.apply_ramp(self.actual_speed_r, self.target_speed_r)

        send_speed(self.actual_speed_l, self.actual_speed_r)

    def apply_ramp(self, current, target):
        diff = target - current
        if abs(diff) <= RAMP_STEP:
            return target
        return current + RAMP_STEP * (1 if diff > 0 else -1)

def main(args=None):
    rclpy.init(args=args)
    node = DiffDriveController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Node mit Ctrl+C beendet")
        send_speed(0, 0)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

