import rclpy
from rclpy.node import Node

from picka_interfaces.srv import CalculateFK
import math

from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

class PickaCLI(Node):
    def __init__(self):
        super().__init__('picka_cli')

        self.fk_client = self.create_client(CalculateFK, 'calculate_fk')

        self.arm_command_publisher = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            10,
        )
    def wait_for_fk_server(self):
        while not self.fk_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for FK server...')

    def request_fk(self, joint_angles):
        request = CalculateFK.Request()
        request.joint_angles = joint_angles

        future = self.fk_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        return future.result()

    def move_gazebo_arm(self, joint_angles_degrees):
        if len(joint_angles_degrees) != 4:
            raise ValueError("Exactly four arm angles are required.")

        trajectory = JointTrajectory()

        trajectory.joint_names = [
            'base_joint',
            'shoulder_joint',
            'elbow_joint',
            'wrist_joint',
        ]

        point = JointTrajectoryPoint()

        # Gazebo controllers require radians.
        point.positions = [
            math.radians(angle)
            for angle in joint_angles_degrees
        ]

        point.time_from_start = Duration(
            sec=3,
            nanosec=0,
        )

        trajectory.points = [point]

        self.arm_command_publisher.publish(trajectory)

        self.get_logger().info(
            f'Gazebo command sent: {joint_angles_degrees} degrees'
    )
def read_joint_angles():
    user_input = input(
        'Enter θ1 θ2 θ3 θ4 in degrees, separated by spaces: '
    )

    values = [float(value) for value in user_input.split()]

    if len(values) != 4:
        raise ValueError(
            'You must enter exactly four arm-joint angles.'
        )

    return values
def main(args=None):
    rclpy.init(args=args)
    node = PickaCLI()
    try:
        print('\n Robotic Arm Kinematics')
        print('1. Forward Kinematics')
        print('2. Inverse Kinematics')
        print('0. Exit')
        choice = input('Select an operation: ').strip()
        if choice == '1':
            joint_angles = read_joint_angles()
            node.wait_for_fk_server()
            response = node.request_fk(joint_angles)
            if response is None:
                print('The FK service returned no response.')
            elif response.success:
                print('\nEnd-effector pose')
                print(f'x     = {response.x:.4f} m')
                print(f'y     = {response.y:.4f} m')
                print(f'z     = {response.z:.4f} m')
                print(f'roll  = {response.roll:.2f} degrees')
                print(f'pitch = {response.pitch:.2f} degrees')
                print(f'yaw   = {response.yaw:.2f} degrees')

                # Send the same four angles to Gazebo
                node.move_gazebo_arm(joint_angles)

                # Give ROS time to transmit the trajectory message
                rclpy.spin_once(node, timeout_sec=0.5)
            else:
                print(f'FK failed: {response.message}')
        elif choice == '2':
            print('The IK service has not been implemented yet.')
        elif choice == '0':
            print('Exiting.')
        else:
            print('Invalid selection.')
    except ValueError as error:
        print(f'Input error: {error}')
    except KeyboardInterrupt:
        print('\nInterrupted by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()
