# from tools import *
# from roblib import *


def f(x1, x2):
    (n, p) = x1.shape
    res_x = np.empty(shape=(n, p))
    res_y = np.empty(shape=(n, p))
    for i in range(n):
        for j in range(p):
            x = x1[i, j]
            y = x2[i, j]
            res = get_force(line, array([[x], [y]]), kd=50, kn=1)
            res_x[i, j], res_y[i, j] = res.flatten()
    return res_x, res_y


if __name__ == '__main__':
    # width = 50
    # step = 10
    # line = Line('plage', 'ouest')
    #
    # print(line.pos0, line.pos1)
    #
    # xmin, xmax, ymin, ymax = -150, 150, 0, 200
    # ax = init_figure(xmin, xmax, ymin, ymax)
    # draw_field(ax, f, xmin, xmax, ymin, ymax, step)
    #
    # pause(20)

    t = [(1, 2), (3, 4)]
    f = open('coucou', 'w')
    f.write(str(t))

