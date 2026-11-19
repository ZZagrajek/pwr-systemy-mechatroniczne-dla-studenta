
"""
Sterowanie żółwiem turtlesim po zadanej liście punktów.

Subskrybuje: /turtle1/pose (turtlesim/msg/Pose)
Publikuje:   /turtle1/cmd_vel (geometry_msgs/msg/Twist)
             /osiagniety_punkt (std_msgs/msg/Int32)

Parametry:
    punkty_x - lista współrzędnych X
    punkty_y - lista współrzędnych Y
    tolerancja - odległość uznawana za osiągnięcie punktu
    K_v - wzmocnienie regulatora prędkości liniowej
    K_w - wzmocnienie regulatora kąta
    v_max - maksymalna prędkość liniowa
    w_max - maksymalna prędkość kątowa
"""

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_msgs.msg import Int32


class DoPunktu(Node):

    def __init__(self):
        super().__init__('do_punktu')

        # Parametry punktów docelowych
        self.declare_parameter('punkty_x', [8.0, 8.0, 3.0, 3.0])
        self.declare_parameter('punkty_y', [3.0, 8.0, 8.0, 3.0])

        # Parametry regulatora
        self.declare_parameter('tolerancja', 0.15)
        self.declare_parameter('K_v', 1.0)
        self.declare_parameter('K_w', 6.0)
        self.declare_parameter('v_max', 2.0)
        self.declare_parameter('w_max', 4.0)

        self.x = None
        self.y = None
        self.theta = None

        self.numer_punktu = 0

        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pub_punkt = self.create_publisher( Int32, '/osiagniety_punkt', 10 )

        self.create_subscription( Pose, '/turtle1/pose', self.na_poze,  10)
        self.create_timer(1.0 / 20.0, self.sterowanie)
        self.get_logger().info('Węzeł do_punktu uruchomiony')

    def na_poze(self, msg: Pose):
        self.x = msg.x
        self.y = msg.y
        self.theta = msg.theta

    def ogranicz(self, wartosc, maksimum):
        return max(-maksimum, min(maksimum, wartosc))

    def sterowanie(self):
        if self.x is None:
            return

        punkty_x = self.get_parameter('punkty_x').value
        punkty_y = self.get_parameter('punkty_y').value

        tolerancja = self.get_parameter('tolerancja').value
        K_v = self.get_parameter('K_v').value
        K_w = self.get_parameter('K_w').value
        v_max = self.get_parameter('v_max').value
        w_max = self.get_parameter('w_max').value

        # Sprawdzenie poprawności list punktów
        if len(punkty_x) != len(punkty_y) or len(punkty_x) == 0:
            self.get_logger().error('Listy punkty_x i punkty_y muszą mieć jednakową,' 'niezerową długość')
            self.pub.publish(Twist())
            return

        # Wszystkie punkty zostały osiągnięte
        if self.numer_punktu >= len(punkty_x):
            self.pub.publish(Twist())
            return

        # Aktualny punkt docelowy
        cel_x = punkty_x[self.numer_punktu]
        cel_y = punkty_y[self.numer_punktu]

        # Odległość do punktu
        dx = cel_x - self.x
        dy = cel_y - self.y

        odleglosc = math.sqrt(dx**2 + dy**2)

        # Osiągnięcie punktu
        if odleglosc < tolerancja:

            self.pub.publish(Twist())

            msg = Int32()
            msg.data = self.numer_punktu + 1
            self.pub_punkt.publish(msg)

            self.get_logger().info(
                f'Osiągnięto punkt {self.numer_punktu + 1}: '
                f'({cel_x:.2f}, {cel_y:.2f})'
            )

            self.numer_punktu += 1
            return

        kat_celu = math.atan2(dy, dx)
        blad_kata = kat_celu - self.theta
        blad_kata = (blad_kata + math.pi) % (2.0 * math.pi) - math.pi
        predkosc_katowa = K_w * blad_kata
        predkosc_liniowa = K_v * odleglosc

        predkosc_liniowa *= max(0.0, math.cos(blad_kata))
        predkosc_liniowa = self.ogranicz(predkosc_liniowa, v_max)

        predkosc_katowa = self.ogranicz(predkosc_katowa, w_max )


        cmd = Twist()  3
        cmd.linear.x = float(predkosc_liniowa)
        cmd.angular.z = float(predkosc_katowa)
        self.pub.publish(cmd)


def main(args=None):

    rclpy.init(args=args)

    wezel = DoPunktu()

    try:
        rclpy.spin(wezel)

    except KeyboardInterrupt:
        pass

    finally:
        wezel.pub.publish(Twist())
        wezel.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
