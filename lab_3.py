import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np
np.set_printoptions(precision=3, suppress=True)

Kp = 0.01     # kp = 3
Kd = 0.1

class InverseKinematics(Node):

    def __init__(self):
        super().__init__('inverse_kinematics')
        self.joint_subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.listener_callback,
            10)
        self.joint_subscription  # prevent unused variable warning

        self.command_publisher = self.create_publisher(
            Float64MultiArray,
            '/forward_command_controller/commands',
            10
        )

        self.pd_timer_period = 1.0 / 200  # 200 Hz     # self.pd_timer_period = 1.0 / 200  # 200 Hz
        self.ik_timer_period = 1.0 / 20   # 10 Hz      # self.ik_timer_period = 1.0 / 20   # 10 Hz
        self.pd_timer = self.create_timer(self.pd_timer_period, self.pd_timer_callback)
        self.ik_timer = self.create_timer(self.ik_timer_period, self.ik_timer_callback)

        self.joint_positions = None
        self.joint_velocities = None
        self.target_joint_positions = None

        self.ee_triangle_positions = np.array([
            [0.05, 0.0, -0.12],  # Touchdown
            [-0.05, 0.0, -0.12], # Liftoff
            [0.0, 0.0, -0.06]    # Mid-swing
        ])

        center_to_rf_hip = np.array([0.07500, -0.08350, 0])
        self.ee_triangle_positions = self.ee_triangle_positions + center_to_rf_hip
        self.current_target = 0
        self.t = 0

    def listener_callback(self, msg):
        joints_of_interest = ['leg_front_r_1', 'leg_front_r_2', 'leg_front_r_3']
        self.joint_positions = np.array([msg.position[msg.name.index(joint)] for joint in joints_of_interest])
        self.joint_velocities = np.array([msg.velocity[msg.name.index(joint)] for joint in joints_of_interest])

    def forward_kinematics(self, theta1, theta2, theta3):

        def rotation_x(angle):
            # rotation about the x-axis implemented for you
            return np.array(
                [
                    [1, 0, 0, 0],
                    [0, np.cos(angle), -np.sin(angle), 0],
                    [0, np.sin(angle), np.cos(angle), 0],
                    [0, 0, 0, 1],
                ]
            )

        def rotation_y(angle):
            return np.array([
                [np.cos(angle), 0, np.sin(angle), 0],
                [0, 1, 0, 0],
                [-np.sin(angle), 0, np.cos(angle), 0],
                [0, 0, 0, 1],
            ])

        def rotation_z(angle):
            return np.array([
                [np.cos(angle), -np.sin(angle), 0, 0],
                [np.sin(angle), np.cos(angle), 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1],
            ])

        def translation(x, y, z):
            ## TODO: Implement the translation matrix
            return np.array([
                [1, 0, 0, x],
                [0, 1, 0, y],
                [0, 0, 1, z],
                [0, 0, 0, 1],
            ])
            raise NotImplementedError()

        # T_0_1 (base_link to leg_front_l_1)
        T_0_1 = translation(0.07500, -0.0445, 0) @ rotation_x(1.57080) @ rotation_z(theta1)

        # T_1_2 (leg_front_l_1 to leg_front_l_2)
        T_1_2 = translation(0,0, 0.03900) @ rotation_y(-1.57080) @ rotation_z(theta2)

        # T_2_3 (leg_front_l_2 to leg_front_l_3)
        T_2_3 = translation(0, -0.0494, 0.0685) @ rotation_y(1.57080) @ rotation_z(theta3)

        # T_3_ee (leg_front_l_3 to end-effector)
        T_3_ee = translation(0.06231, -0.06216, 0.018)

        T_0_ee = T_0_1 @ T_1_2 @ T_2_3 @ T_3_ee

        end_effector_position = T_0_ee[:3, -1]

        return end_effector_position

    def inverse_kinematics(self, target_ee, initial_guess=[0, 0, 0]):
        def cost_function(theta):
            # Compute the cost function and the squared L2 norm of the error
            # return the cost and the squared L2 norm of the error
            ################################################################################################
            # TODO: Implement the cost function
            # HINT: You can use the * notation on a list to "unpack" a list
            ################################################################################################
            x, y, z = theta
            current_ee = self.forward_kinematics(x, y, z)
            distance = np.abs(current_ee - target_ee)
            cost = 0
            for i in range(len(distance)):
                cost += (distance[i]**2)
                distance[i] = (distance[i]**2)
            return cost, distance

        def gradient(theta, epsilon=1e-3):
            # Compute the gradient of the cost function using finite differences
            ################################################################################################
            # TODO: Implement the gradient computation
            ################################################################################################
            ep_array = [epsilon, epsilon, epsilon]
            _, distance1 = cost_function(theta + ep_array)
            _, distance2 = cost_function(theta - ep_array)
            num = distance1 - distance2
            denom = 2 * epsilon
            return num / denom

        theta = np.array(initial_guess)
        learning_rate = 10 # TODO: Set the learning rate
        max_iterations = 20 # TODO: Set the maximum number of iterations
        tolerance = 0.0001 # TODO: Set the tolerance for the L1 norm of the error

        cost_l = []
        for _ in range(max_iterations):
            grad = gradient(theta)

            # Update the theta (parameters) using the gradient and the learning rate
            ################################################################################################
            # TODO: Implement the gradient update. Use the cost function you implemented, and use tolerance t
            # to determine if IK has converged
            # TODO (BONUS): Implement the (quasi-)Newton's method instead of finite differences for faster convergence
            ################################################################################################
            new_theta = theta - learning_rate * grad
            cost, distance = cost_function(new_theta)
            if cost < tolerance:
                break
            theta = new_theta

        # print(f'Cost: {cost_l}') # Use to debug to see if you cost function converges within max_iterations

        return theta

    def interpolate_triangle(self, t):
        # Intepolate between the three triangle positions in the self.ee_triangle_positions
        # based on the current time t
        ################################################################################################
        # TODO: Implement the interpolation function
        ################################################################################################
        touchdown, liftoff, midswing = self.ee_triangle_positions

        t_mod = t % 3
        if 0 <= t_mod < 1.0:
            return [touchdown[0] + t_mod * (liftoff[0] - touchdown[0]), touchdown[1] + t_mod * (liftoff[1] - touchdown[1]), touchdown[2] + t_mod * (liftoff[2] - touchdown[2])]
        elif 1.0 <= t_mod < 2.0:
            t_frame = t_mod - 1
            return [liftoff[0] + t_frame * (midswing[0] - liftoff[0]), liftoff[1] + t_frame * (midswing[1] - liftoff[1]), liftoff[2] + t_frame * (midswing[2] - liftoff[2])]
        else:
            t_frame = t_mod - 2
            return [midswing[0] + t_frame * (touchdown[0] - midswing[0]), midswing[1] + t_frame * (touchdown[1] - midswing[1]), midswing[2] + t_frame * (touchdown[2] - midswing[2])]

    def ik_timer_callback(self):
        if self.joint_positions is not None:
            # figures out place where it should go
            target_ee = self.interpolate_triangle(self.t)

            # figures out joint angles to get there
            self.target_joint_positions = self.inverse_kinematics(target_ee, self.joint_positions)

            # figure out where we are
            current_ee = self.forward_kinematics(*self.joint_positions)

            # update the current time for the triangle interpolation
            ################################################################################################
            # TODO: Implement the time update
            ################################################################################################
            self.t = self.t + self.ik_timer_period
            

            self.get_logger().info(f'Target EE: {target_ee}, Current EE: {current_ee}, Target Angles: {self.target_joint_positions}, Target Angles to EE: {self.forward_kinematics(*self.target_joint_positions)}, Current Angles: {self.joint_positions}')

    def pd_timer_callback(self):
        if self.target_joint_positions is not None:

            command_msg = Float64MultiArray()
            command_msg.data = self.target_joint_positions.tolist()
            self.command_publisher.publish(command_msg)

def main():
    rclpy.init()
    inverse_kinematics = InverseKinematics()
    
    try:
        rclpy.spin(inverse_kinematics)
    except KeyboardInterrupt:
        print("Program terminated by user")
    finally:
        # Send zero torques
        zero_torques = Float64MultiArray()
        zero_torques.data = [0.0, 0.0, 0.0]
        inverse_kinematics.command_publisher.publish(zero_torques)
        
        inverse_kinematics.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
