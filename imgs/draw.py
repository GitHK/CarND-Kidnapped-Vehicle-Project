import matplotlib.pyplot as plt

# tuples (particles, x_error, y_error, yaw_error, system_time)
IMG_DATA = [
    (5, .219, .194, .007, 58.4),
    (10, .163, .148, .005, 58.42),
    (15, .143, .132, .005, 58.12),
    (20, .136, .130, .005, 58.66),
    (50, .121, .112, .004, 57.82),
    (100, .117, .106, .004, 58.2),
    (1000, .109, .097, .003, 67.72)
]


def get_data(index):
    return [x[index] for x in IMG_DATA]


def get_particles():
    return get_data(0)


def make_error_raph():
    fig, ax = plt.subplots()

    ax.plot(get_data(0), get_data(1), ms=20, lw=2, alpha=0.7, label='x_error')
    ax.plot(get_data(0), get_data(2), ms=20, lw=2, alpha=0.7, label='y_error')
    ax.plot(get_data(0), get_data(3), ms=20, lw=2, alpha=0.7, label='yaw_error')
    ax.grid()
    ax.legend()

    plt.xlabel('particles')

    plt.savefig("plotted_errors.png")


def make_time_graph():
    fig, ax = plt.subplots()

    ax.plot(get_data(0), get_data(4), ms=20, lw=2, alpha=0.7, label='time')
    ax.grid()
    ax.legend()

    axes = plt.gca()
    axes.set_ylim([56, 70])

    plt.xlabel('particles')

    plt.savefig("plotted_time.png")


if __name__ == '__main__':
    make_error_raph()
    make_time_graph()
