import rclpy
from rclpy.node import Node

from picka_interfaces.srv import CalculateFK

class PickaCLI(Node):
    def __init__(self):
        super().__init__('picka_cli')

        self.fk_client = self.create_client(CalculateFK, 'calculate_fk')

    def wait_for_fk_server(self):
        while not self.fk_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for FK server...')

    def request_fk(self, joint_angles):
        request = CalculateFK.Request()
        request.joint_angles = joint_angles

        future = self.fk_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        return future.result()
def read_joint_angles():
    user_input = input(
        'Enter θ1 θ2 θ3 θ4 θ5 in degrees, separated by spaces: '
    )
    values = [float(value) for value in user_input.split()]
    if len(values) != 5:
        raise ValueError('You must enter exactly five joint angles.')
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
                print(f'x   = {response.x:.4f} m')
                print(f'y   = {response.y:.4f} m')
                print(f'z   = {response.z:.4f} m')
                print(f'roll: = {response.roll:.2f} degree')
                print(f'pitch = {response.pitch:.2f} degeree')
                print(f'yaw = {response.yaw:.2f} degree')
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
