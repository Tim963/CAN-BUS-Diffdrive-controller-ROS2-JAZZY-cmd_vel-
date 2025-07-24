import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import can

# === Plattformparameter ===
WHEEL_DISTANCE = 1.25  # Meter – Spurweite des Roboters
SPEED_SCALE = 1000     # Skalierungsfaktor m/s Motorgeschwindigkeitseinheit

# === CAN-Bus Setup ===
can_interface = 'can0'
bus = can.Bus(interface='socketcan', channel=can_interface)

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

class TwistStampedToCAN(Node):
    def __init__(self):
        super().__init__('twiststamped_to_can')
        self.subscription = self.create_subscription(
            TwistStamped,
            '/cmd_vel',
            self.listener_callback,
            10)
        self.get_logger().info('Node gestartet: /cmd_vel (TwistStamped) → CAN')

    def listener_callback(self, msg: TwistStamped):
        v = -msg.twist.linear.x
        omega = -msg.twist.angular.z

        # Differential Drive Umrechnung
        v_l = v - (omega * WHEEL_DISTANCE / 2)
        v_r = v + (omega * WHEEL_DISTANCE / 2)

        speed_l = int(v_l * SPEED_SCALE)
        speed_r = int(v_r * SPEED_SCALE)

        send_speed(speed_l, speed_r)

def main(args=None):
    rclpy.init(args=args)
    node = TwistStampedToCAN()
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
