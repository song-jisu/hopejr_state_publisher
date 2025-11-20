import numpy as np
from scipy.optimize import minimize
from scipy.optimize import least_squares

start1 = np.array([6.9, 6.7, 0])
start2 = np.array([6.9, -6.7, 0])

end1_init = np.array([0, 6.7, 28])
end2_init = np.array([0, -6.7, 28])

L1_init = np.linalg.norm(end1_init - start1)
L2_init = np.linalg.norm(end2_init - start2)
bar_len_init = np.linalg.norm(end1_init - end2_init)
bar_center_init = (end1_init + end2_init) / 2

def bar_points(theta1, theta2):
    L1 = L1_init - 9 * theta1
    L2 = L2_init + 9 * theta2

    def cost(x):
        end1, end2 = x[:3], x[3:]
        center = (end1 + end2) / 2
        return np.linalg.norm(center - bar_center_init)

    def cons1(x):
        end1 = x[:3]
        return np.linalg.norm(end1 - start1) - L1

    def cons2(x):
        end2 = x[3:]
        return np.linalg.norm(end2 - start2) - L2

    def cons3(x):
        end1, end2 = x[:3], x[3:]
        return np.linalg.norm(end1 - end2) - bar_len_init

    x0 = np.concatenate([end1_init, end2_init])
    constraints = [
        {'type': 'eq', 'fun': cons1},
        {'type': 'eq', 'fun': cons2},
        {'type': 'eq', 'fun': cons3}
    ]
    res = minimize(cost, x0, constraints=constraints)
    end1_opt = res.x[:3]
    end2_opt = res.x[3:]
    bar_center_opt = (end1_opt + end2_opt) / 2
    bar_len = np.linalg.norm(end1_opt - end2_opt)
    return end1_opt, end2_opt, bar_center_opt, bar_len

def thetas_to_alphabeta(theta1, theta2):
    end1, end2, bar_center, bar_len = bar_points(theta1, theta2)
    def error(vars):
        alpha, beta = vars
        # Real 1 end point model
        X1 = np.sin(alpha) * (28.19 * np.cos(beta) + 7.6 * np.sin(beta) - 7.6)
        Y1 = -28.19 * np.sin(beta) + 7.6 * np.cos(beta)
        Z1 = np.cos(alpha) * (28.19 * np.cos(beta) + 7.6 * np.sin(beta) - 7.6) + 4.1

        # Real 2 end point model
        X2 = np.sin(alpha) * (28.19 * np.cos(beta) - 7.6 * np.sin(beta) - 7.6)
        Y2 = -28.19 * np.sin(beta) - 7.6 * np.cos(beta)
        Z2 = np.cos(alpha) * (28.19 * np.cos(beta) - 7.6 * np.sin(beta) - 7.6) + 4.1

        e1 = np.array([X1, Y1, Z1]) - end1
        e2 = np.array([X2, Y2, Z2]) - end2
        # Sum of the errors of the two end points
        return np.hstack((e1, e2))

    # Initial value and bounds
    x0 = [np.radians(90), 0]
    bounds = ([np.radians(0), np.radians(-90)], [np.radians(90), np.radians(90)])
    res = least_squares(error, x0, bounds=bounds)
    alpha, beta = res.x[0], -res.x[1]
    return alpha, beta
# Example
theta1, theta2 = 1, -3
end1, end2, bar_center, bar_len = bar_points(theta1, theta2)
alpha, beta = thetas_to_alphabeta(theta1, theta2)
print("Real 1 end point:", end1)
print("Real 2 end point:", end2)
print("Bar center:", bar_center)
print("Bar length:", bar_len)
print("alpha:", alpha)
print("beta:", beta)
