import numpy
import numpy as np
from numpy import sin, cos, arctan2
import matplotlib.pyplot as plt


# ** RETURN VALUE ** 'joint_positions' is a list contains joint state in the form as following:
# [position_0, position1_0, position2_0]
# each elements position_i is a 4 orders 'numpy.ndarray', which specify a column vector
#  ie:
#    position_0 = np.array([0, 0, 0, 1]).T

l_1 = 0.2284
l_2 = 0.1897
d_1 = 0
d_2 = 0
alpha_1 = 0
alpha_2 = 0


def dh_matrix(delta, d, a, alpha):  # transform matrix of DH
    return np.array([[cos(delta), -sin(delta) * cos(alpha), sin(delta) * sin(alpha), a * cos(delta)],
                     [sin(delta), cos(delta) * cos(alpha), -cos(delta) * sin(alpha), a * sin(delta)],
                     [0, sin(alpha), cos(alpha), d],
                     [0, 0, 0, 1]])


def fk(theta_1, theta_2):
    position_0 = np.array([0, 0, 0, 1]).T  # transpose, column vector
    position_1 = np.array([0, 0, 0, 1]).T
    position_2 = np.array([0, 0, 0, 1]).T

    position1_0 = dh_matrix(theta_1, d_1, l_1, alpha_1).dot(position_1)  # results of end of link1 position
    position2_0 = dh_matrix(theta_1, d_1, l_1, alpha_1).dot(dh_matrix(theta_2, d_2, l_2, alpha_2)).dot(
        position_2)  # link2
    return [position_0, position1_0, position2_0]  # positions of links' end


def position_plot(joint_positions, des=None, is_from_to=False):
    fig, ax = plt.subplots()
    if not is_from_to:
        ax.plot([joint_positions[0][0], joint_positions[1][0], joint_positions[2][0]],
                [joint_positions[0][1], joint_positions[1][1], joint_positions[2][1]], '-bo')
        print('[INFO] position of joint0:', joint_positions[0][0:2])
        print('[INFO] position of joint1:', joint_positions[1][0:2])
        print('[INFO] position of joint2:', joint_positions[2][0:2])
    else:
        joint_positions_list = joint_positions
        for i in range(len(joint_positions_list)):
            ax.plot([joint_positions_list[i][0][0], joint_positions_list[i][1][0], joint_positions_list[i][2][0]],
                    [joint_positions_list[i][0][1], joint_positions_list[i][1][1],
                     joint_positions_list[i][2][1]], '-bo')
            print('[INFO] position of joint0:', joint_positions_list[i][0][0:2])
            print('[INFO] position of joint1:', joint_positions_list[i][1][0:2])
            print('[INFO] position of joint2:', joint_positions_list[i][2][0:2])
    if des is not None:
        ax.plot(des[0], des[1], '-rx')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    plt.xlim(0, 0.5)
    plt.ylim(0, 0.5)
    fig.savefig('./pic/temp.pdf')
    plt.show()


def fk_solver(theta_1, theta_2, show):  # theta_2 multiply a minus here because we use upper elbow
    print('>>> RUNNING INTO FK_SOLVER')
    theta_1_rad = np.radians(theta_1)
    theta_2_rad = np.radians(-theta_2)
    print('[INFO] Input [theta_1 theta_2] in deg:', [theta_1, -theta_2])
    joint_positions = fk(theta_1_rad, theta_2_rad)
    if show:
        position_plot(joint_positions)
    return joint_positions


def ik(des_position_2_0, show_error):
    position_0 = np.array([0, 0, 0, 1]).T
    position_1 = np.array([0, 0, 0, 1]).T
    position_2 = np.array([0, 0, 0, 1]).T

    D = (des_position_2_0[0] ** 2 + des_position_2_0[1] ** 2 - l_1 ** 2 - l_2 ** 2) / (2 * l_1 * l_2)
    theta_2 = arctan2(-(1 - D ** 2) ** 0.5, D)  # note that here use a upper elbow state
    theta_1 = arctan2(des_position_2_0[1], des_position_2_0[0]) - arctan2(l_2 * sin(theta_2), l_1 + l_2 * cos(theta_2))

    print('[INFO] Output [theta_1 theta_2] in deg:', numpy.degrees([theta_1, theta_2]))

    position1_0 = dh_matrix(theta_1, d_1, l_1, alpha_1).dot(position_1)
    position2_0 = dh_matrix(theta_1, d_1, l_1, alpha_1).dot(dh_matrix(theta_2, d_2, l_2, alpha_2)).dot(position_2)
    return [position_0, position1_0, position2_0]  # positions of links' end


def ik_solver(des_position_2_0, show):
    print('>>> RUNNING INTO IK_SOLVER')
    print('[INFO] destination of the end:', des_position_2_0)
    joint_positions = ik(des_position_2_0, True)
    if show:
        position_plot(joint_positions, des=des_position_2_0)
    return joint_positions


def plot_from_to(from_position, to_position):
    generated_positions = list()
    generated_positions.append(np.linspace(from_position[0], to_position[0], 10))
    print(generated_positions[0])
    generated_positions.append(np.linspace(from_position[1], to_position[1], 10))
    print(generated_positions[1])
    joint_positions_list = list()
    for i in range(len(generated_positions[0])):
        joint_positions_list.append(ik_solver([generated_positions[0][i], generated_positions[1][i]], False))
    position_plot(joint_positions_list, is_from_to=True)


# solver could be used in project
fk_solver(20, 30, True)
ik_solver([0.3, 0.03], True)

# simulate the progress
plot_from_to([0.3, 0.03], [0.3, 0.2])
plot_from_to([0.3, 0.2], [0.2, 0.2])
plot_from_to([0.2, 0.2], [0.2, 0.03])
