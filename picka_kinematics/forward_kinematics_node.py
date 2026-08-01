import numpy as np


import numpy as np


# Four actuated arm joints
DH_PARAMETERS = [
    # a(m), alpha(deg), d(m), theta_offset(deg)
    [0.001750,   0.0, 0.068622,    0.0],   # Joint 1
    [0.012119, -90.0, 0.101580,  -90.0],   # Joint 2
    [0.095946,   0.0, 0.023349,  123.0],   # Joint 3
    [0.011960, -90.0, 0.009900, -161.0],   # Joint 4
]

# Fixed transformation from Joint 4 to the tool/end-effector.
# This was previously the fifth DH row, but theta is no longer variable.
TOOL_DH_PARAMETER = [0.009913, 0.0, 0.155697, 0.0]

JOINT_SIGNS = [1, 1, 1, 1]

JOINT_LIMITS = [
    (-160.0, 160.0),
    (-45.0, 90.0),
    (0.0, 120.0),
    (-80.0, 80.0),
]
def standard_dh_transform(a, alpha, d, theta):

    ct = np.cos(theta)
    st = np.sin(theta)
    ca = np.cos(alpha)
    sa = np.sin(alpha)


    return np.array([
        [ct,    -st*ca,     st*sa,      a*ct],
        [st,    ct*ca,      -ct*sa,     a*st],
        [0,     sa,         ca,         d],
        [0,     0,          0,          1]
    ], dtype=float)

def standard_dh_transform(a, alpha, d, theta):
    ct = np.cos(theta)
    st = np.sin(theta)
    ca = np.cos(alpha)
    sa = np.sin(alpha)

    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0.0,      sa,       ca,      d],
        [0.0,     0.0,      0.0,    1.0],
    ], dtype=float)


def forward_kinematics(
        joint_angles,
        dh_parameters=DH_PARAMETERS,
        joint_signs=JOINT_SIGNS,
        angles_in_degrees=True,
    ):
        number_of_joints = len(dh_parameters)

        if len(joint_angles) != number_of_joints:
            raise ValueError(
                f"Expected {number_of_joints} arm-joint angles, "
                f"but received {len(joint_angles)}."
            )

        if len(joint_signs) != number_of_joints:
            raise ValueError(
                "joint_signs must contain one sign per arm joint."
            )

        transform = np.eye(4)
        transformations = []

        # Four variable arm-joint transformations
        for i, (a, alpha, d, theta_offset) in enumerate(dh_parameters):
            theta = (
                joint_signs[i] * float(joint_angles[i])
                + theta_offset
            )

            if angles_in_degrees:
                alpha = np.deg2rad(alpha)
                theta = np.deg2rad(theta)

            joint_transform = standard_dh_transform(
                a=a,
                alpha=alpha,
                d=d,
                theta=theta,
            )

            transform = transform @ joint_transform
            transformations.append(transform.copy())

        # Fixed Joint-4-to-tool transformation
        tool_a, tool_alpha, tool_d, tool_theta = TOOL_DH_PARAMETER

        if angles_in_degrees:
            tool_alpha = np.deg2rad(tool_alpha)
            tool_theta = np.deg2rad(tool_theta)

        tool_transform = standard_dh_transform(
            a=tool_a,
            alpha=tool_alpha,
            d=tool_d,
            theta=tool_theta,
        )

        end_effector_transform = transform @ tool_transform

        return end_effector_transform, transformations
if __name__ == "__main__":
    q_degrees = [30.0, 20.0, 40.0, 15.0]

    T_0_E, frame_transforms = forward_kinematics(q_degrees)

    position = T_0_E[:3, 3]
    rotation_matrix = T_0_E[:3, :3]

    print("End-effector transformation T_0_E:")
    print(np.round(T_0_E, 5))

    print("\nEnd-effector position [x, y, z] in metres:")
    print(np.round(position, 5))

    print("\nEnd-effector rotation matrix:")
    print(np.round(rotation_matrix, 5))