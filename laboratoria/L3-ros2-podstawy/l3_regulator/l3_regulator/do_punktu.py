
import math
import rclpy

from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_msgs.msg import Int32


class DoPunktu(Node):

    def __init__(self):
        super().__init__('do_punktu')

        # Parametry
        self.declare_parameter('punkty_x', [8.0, 8.0, 3.0, 3.0])
        self.declare_parameter('punkty_y', [3.0, 8.0, 8.0, 3.0])
        self.declare_parameter('tolerancja', 0.15)
        self.declare_parameter('K_v', 1.0)
        self.declare_parameter('K_w', 6.0)
        self.declare_parameter('v_max', 2.0)
        self.declare_parameter('w_max', 4.0)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.punkt = 0

        self.create_subscription(Pose, '/turtle1/pose', self.odbierz_pozycje, 10)

        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        self.pub_punkt = self.create_publisher(Int32, '/osiagniety_punkt', 10)

         self.create_timer(1.0 / 20.0, self.sterowanie)

    def odbierz_pozycje(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.theta = msg.theta

    def sterowanie(self):

        px = self.get_parameter('punkty_x').value
        py = self.get_parameter('punkty_y').value

        if self.punkt >= len(px):
            self.pub.publish(Twist())
            return

        # Odległość do punktu
        dx = px[self.punkt] - self.x
        dy = py[self.punkt] - self.y
        odleglosc = math.sqrt(dx**2 + dy**2)

        tolerancja = self.get_parameter('tolerancja').value

        if odleglosc < tolerancja:            # Sprawdzenie osiągnięcia punktu
            self.pub.publish(Twist())

            msg = Int32()
            msg.data = self.punkt + 1
            self.pub_punkt.publish(msg)

            self.get_logger().info(
                f'Osiągnięto punkt {self.punkt + 1}')

            self.punkt += 1
            return

        kat = math.atan2(dy, dx)            # Kąt do celu

        blad = (kat - self.theta + math.pi) % (2 * math.pi) - math.pi       # Błąd kąta [-pi, pi)

        Kv = self.get_parameter('K_v').value            # Regulator P
        Kw = self.get_parameter('K_w').value

        v_max = self.get_parameter('v_max').value
        w_max = self.get_parameter('w_max').value

        v = Kv * odleglosc * max(0.0, math.cos(blad))
        w = Kw * blad

        v = max(-v_max, min(v_max, v))  # Ograniczenie prędkości
        w = max(-w_max, min(w_max, w))

        cmd = Twist()                   # Publikacja prędkości
        cmd.linear.x = float(v)
        cmd.angular.z = float(w)

        self.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = DoPunktu()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

