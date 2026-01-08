#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


def quat_to_rpy(x, y, z, w):
    # ROS quaternion (x,y,z,w) -> roll, pitch, yaw
    # https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = max(min(t2, 1.0), -1.0)
    pitch = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)

    return roll, pitch, yaw


class OdomRampEstimator(Node):
    def __init__(self):
        super().__init__('odom_ramp_estimator')

        # Params
        self.declare_parameter('odom_in', '/skid_drive_controller/odom')
        self.declare_parameter('imu_in', '/imu')
        self.declare_parameter('odom_out', '/odom_ramp')
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'base_link')
        self.declare_parameter('use_vx_sign', True) 

        self.declare_parameter('pitch_lpf_alpha', 0.2)   # 0..1 (maior = menos filtro)
        self.declare_parameter('min_speed_mps', 0.02)    # trava z quando muito lento
        self.declare_parameter('z_max_step', 0.20)       # clamp por callback (segurança)

        self.odom_in = str(self.get_parameter('odom_in').value)
        self.imu_in = str(self.get_parameter('imu_in').value)
        self.odom_out = str(self.get_parameter('odom_out').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.child_frame_id = str(self.get_parameter('child_frame_id').value)

        self.alpha = float(self.get_parameter('pitch_lpf_alpha').value)
        self.min_speed = float(self.get_parameter('min_speed_mps').value)
        self.z_max_step = float(self.get_parameter('z_max_step').value)
        self.use_vx_sign = bool(self.get_parameter('use_vx_sign').value)

        # State
        self.have_imu = False
        self.pitch_f = 0.0

        self.have_prev_odom = False
        self.prev_x = 0.0
        self.prev_y = 0.0
        self.z_est = 0.0

        # Subs/Pubs
        self.sub_imu = self.create_subscription(Imu, self.imu_in, self.cb_imu, 50)
        self.sub_odom = self.create_subscription(Odometry, self.odom_in, self.cb_odom, 50)
        self.pub = self.create_publisher(Odometry, self.odom_out, 10)

        self.get_logger().info(
            f'Estimating z from pitch(/imu) + planar odom({self.odom_in}). Publishing {self.odom_out}'
        )

    def cb_imu(self, msg: Imu):
        q = msg.orientation
        _, pitch, _ = quat_to_rpy(q.x, q.y, q.z, q.w)

        # Low-pass filter
        if not self.have_imu:
            self.pitch_f = pitch
            self.have_imu = True
        else:
            self.pitch_f = (1.0 - self.alpha) * self.pitch_f + self.alpha * pitch*-1

    def cb_odom(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        if not self.have_prev_odom:
            self.prev_x, self.prev_y = x, y
            self.have_prev_odom = True
            # Inicializa z com 0 (ou poderia ler de param)
            self.publish(msg, dz=0.0)
            return

        dx = x - self.prev_x
        dy = y - self.prev_y
        ds = math.sqrt(dx * dx + dy * dy)

        self.prev_x, self.prev_y = x, y

        # Se não tem IMU ainda, não mexe em z
        if not self.have_imu:
            self.publish(msg, dz=0.0)
            return

        # Usa velocidade do odom pra travar em baixas velocidades
        v = msg.twist.twist.linear.x
        if abs(v) < self.min_speed:
            self.publish(msg, dz=0.0)
            return
        
        dir_sign = 1.0
        if self.use_vx_sign:
            dir_sign = 1.0 if v >= 0.0 else -1.0

        dz = ds * math.sin(self.pitch_f) * dir_sign

        # clamp para evitar spikes
        if dz > self.z_max_step:
            dz = self.z_max_step
        elif dz < -self.z_max_step:
            dz = -self.z_max_step

        self.z_est += dz
        self.publish(msg, dz=dz)

    def publish(self, odom_in: Odometry, dz: float):
        odom = Odometry()
        odom.header = odom_in.header
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id

        # Copia pose/twist do odom original
        odom.pose = odom_in.pose
        odom.twist = odom_in.twist

        # Injeta z estimado
        odom.pose.pose.position.z = float(self.z_est)

        # (Opcional) você pode aumentar a covariância em z aqui se quiser
        self.pub.publish(odom)


def main():
    rclpy.init()
    node = OdomRampEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
