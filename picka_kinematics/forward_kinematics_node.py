import numpy as np


DH_PARAMETERS = [
    # a(m), alpha(deg), d(m), theta_offset(deg)
    [1.75,  0, 68.622, 0.0],   # Joint 1
    [12.119,   -90, 101.58, -90],   # Joint 2
    [95.946,   0.0, 23.349, 123],   # Joint 3
    [11.96,  -90, 9.9, -161],   # Joint 4
    [9.913,   0.0, 155.697, 0.0]    # Joint 5
    ]
def standard_dh_transform(a, alpha, d, theta):
    dh_parameters = DH_PARAMETERS

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


def forward_kinematics(joint_angles, dh_parameters, joint_signs=None,angles_in_degrees=True):
    number_of_joints = len(dh_parameters)
    if len(joint_angles) != number_of_joints:
        raise ValueError(
            f"Expected {number_of_joints} joint angles, "
            f"but received {len(joint_angles)}."
        )
    if joint_signs is None:
        joint_signs = np.ones(number_of_joints)

    if len(joint_signs) != number_of_joints:
        raise ValueError("joint_signs must contain one value per joint.")

    # Begin at the base coordinate frame
    T_0_n = np.eye(4)
    transformations = []
    for i, (a, alpha, d, theta_offset) in enumerate(dh_parameters):
        joint_angle = joint_signs[i] * joint_angles[i]

        if angles_in_degrees:
            alpha = np.deg2rad(alpha)
            theta = np.deg2rad(joint_angle + theta_offset)
        else:
            theta = joint_angle + theta_offset

        T_previous_current = standard_dh_transform(
            a=a,
            alpha=alpha,
            d=d,
            theta=theta
        )

        T_0_n = T_0_n @ T_previous_current
        transformations.append(T_0_n.copy())

    return T_0_n, transformations

if __name__ == "__main__":
    q_degrees = [30.0, 20.0, -40.0, 15.0, 10.0]

    # Use +1 when the physical positive direction agrees with the
    # positive direction used in the DH model.
    JOINT_SIGNS = [1, 1, 1, 1, 1]

    T_0_5, frame_transforms = forward_kinematics(
        joint_angles=q_degrees,
        dh_parameters=DH_PARAMETERS,
        joint_signs=JOINT_SIGNS,
        angles_in_degrees=True
    )

    position = T_0_5[:3, 3]
    rotation_matrix = T_0_5[:3, :3]

    print("End-effector transformation T_0_5:")
    print(np.round(T_0_5, 5))

    print("\nEnd-effector position [x, y, z] in metres:")
    print(np.round(position, 5))

    print("\nEnd-effector rotation matrix:")
    print(np.round(rotation_matrix, 5))