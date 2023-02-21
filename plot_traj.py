from matplotlib import pyplot as plt

if __name__ == '__main__':
    """
    Scatter trajectories
    """
    f = open("traj_eval2.txt", 'r')
    x = []
    y = []
    for line in f.readlines()[20:]:
        xs, ys = line.split(';')
        x.append(float(xs))
        y.append(float(ys))

    plt.scatter(x, y)
    plt.show()
