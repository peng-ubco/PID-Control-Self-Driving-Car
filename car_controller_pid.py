import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.gridspec as gridspec
import matplotlib.patches as mpatches
import time
import motion_models

# Simulator options.
sim_opt = {}
sim_opt['FIG_SIZE'] = [8, 8]


motion_model = motion_models.motion_model_3

def run_sim(sim_opt):
    start = time.process_time()
    # Simulator Options
    FIG_SIZE = sim_opt['FIG_SIZE']  # [Width, Height]

    # initialize the control input
    num_inputs = 2
    u_solution = np.zeros(num_inputs)

    # reference or set point
    ref = [8, 0, 0]
    state_i = np.array([[1, 0, 0, 0]])
    u_i = np.array([[0, 0]])
    sim_total = 500
    dt = 0.2  # Total simulation time = sim_total * dt

    '''your code here'''
    '''***********************************************'''
    e = np.zeros(sim_total + 1)
    e_dot = np.zeros(sim_total + 1)
    e_int = np.zeros(sim_total + 1)

    # Tune the constants
    K_p = 0.3
    K_d = 0.3
    K_i = 0.1
    '''***********************************************'''

    for i in range(1, sim_total+1):
       e[i - 1] = ref[0] - state_i[-1][0]
       if i > 1:
           e_dot[i - 1] = (e[i - 1] - e[i - 2]) / dt
           e_int[i - 1] = e_int[i - 2] + (e[i - 2] + e[i - 1]) / 2 * dt

           # if e_int[i - 1] > 1: 
           #      e_int[i - 1] = 1
           # if e_int[i-1] < -1:
           #      e_int[i - 1] = -1

       u_solution[0] = K_p * e[i - 1] + K_d * e_dot[i - 1] + K_i * e_int[i - 1]
       u = u_solution
       y = motion_model(state_i[-1], dt, u[0], u[1])
       if i < 11:
            print(u[0])
       state_i = np.append(state_i, np.array([y]), axis=0)
       u_i = np.append(u_i, np.array([(u[0], u[1])]), axis=0)
       # print(i)

    ###################
    # DISPLAY
    fig = plt.figure(figsize=(FIG_SIZE[0], FIG_SIZE[1]))
    gs = gridspec.GridSpec(8, 8)

    # Plot settings.
    ax = fig.add_subplot(gs[:8, :8])

    plt.xlim(-3, 15)
    ax.set_ylim([-3, 15])
    plt.xticks(np.arange(0, 21, step=2))
    plt.yticks(np.arange(0, 21, step=2))

    # Main plot info.
    car_width = 1.5
    car = mpatches.Rectangle((0, 0), car_width, 2.5, fc='m', fill=True)
    goal = mpatches.Rectangle((0, 0), car_width, 2.5, fc='r', ls='dashdot', fill=False)

    ax.add_patch(car)
    ax.add_patch(goal)
    predict, = ax.plot([], [], 'r--', linewidth=1)

    # Car steering and Pedal position.
    loc = [14, 14]
    wheel_loc = [loc[0] - 2, loc[1] + 2]
    pedal_loc = [loc[0] - 5, loc[1] + 3]
    wheel = mpatches.Circle((wheel_loc[0], wheel_loc[1]), 2.2, fill=False, fc='r')
    ax.add_patch(wheel)
    wheel = mpatches.Circle((wheel_loc[0], wheel_loc[1]), 2.4, fill=False, fc='r')
    ax.add_patch(wheel)
    wheel_1, = ax.plot([], [], 'b', linewidth=2)
    wheel_2, = ax.plot([], [], 'b', linewidth=2)
    wheel_3, = ax.plot([], [], 'b', linewidth=2)
    pedal_outline, = ax.plot([loc[0], loc[0]], [loc[1] - 2, loc[1] - 1],
                         'b', linewidth=20, alpha=0.4)
    pedal, = ax.plot([], [], 'k', linewidth=20)
    brake_outline, = ax.plot([loc[0] + 3, loc[0] + 3], [loc[1] - 2, loc[1] - 1],
                         'b', linewidth=20, alpha=0.2)
    brake, = ax.plot([], [], 'k', linewidth=20)
    pedal_text = ax.text(loc[0], loc[1] - 4, 'Forward', fontsize=13,
                     horizontalalignment='center')
    brake_text = ax.text(loc[0] + 4, loc[1] - 4, 'Reverse', fontsize=13,
                     horizontalalignment='center')

    # Speed Indicator
    velocity_text = ax.text(loc[0] + 2, loc[1] + 3, '0', fontsize=10)
    velocity_units_text = ax.text(loc[0] + 3.5, loc[1] + 3, 'km/h', fontsize=10)


    # Shift x y, centered on the rear left corner of car.
    def car_pos(x, y, psi):
        x_new = x - np.sin(psi) * (car_width / 2)
        y_new = y + np.cos(psi) * (car_width / 2)
        return [x_new, y_new]


    def wheel_steering(wheel_angle):
        wheel_1.set_data([wheel_loc[0], wheel_loc[0] + np.cos(wheel_angle) * 2],
                         [wheel_loc[1], wheel_loc[1] + np.sin(wheel_angle) * 2])
        wheel_2.set_data([wheel_loc[0], wheel_loc[0] - np.cos(wheel_angle) * 2],
                         [wheel_loc[1], wheel_loc[1] - np.sin(wheel_angle) * 2])
        wheel_3.set_data([wheel_loc[0], wheel_loc[0] + np.sin(wheel_angle) * 2],
                         [wheel_loc[1], wheel_loc[1] - np.cos(wheel_angle) * 2])


    def plot_update(num):
        # Car.
        car.set_xy(car_pos(state_i[num, 0], state_i[num, 1], state_i[num, 2]))
        car.angle = np.rad2deg(state_i[num, 2]) - 90

        # Car wheels
        wheel_steering(u_i[num, 1] * 2)
        pedal.set_data([loc[0], loc[0]],
                       [loc[1] - 2, loc[1] - 2 + max(0, u_i[num, 0] / 5 * 2)])
        brake.set_data([loc[0] + 3, loc[0] + 3],
                       [loc[1] - 2, loc[1] - 2 + max(0, -u_i[num, 0] / 5 * 2)])

        speed = state_i[num, 3] * 3.6
        velocity_text.set_text(str(round(speed, 1)))
        if speed > 10.1:
            velocity_text.set_color('r')
        else:
            velocity_text.set_color('b')

        # Goal.
        goal.set_xy(car_pos(ref[0], ref[1], ref[2]))
        goal.angle = np.rad2deg(ref[2]) - 90

        return car

    print("Compute Time: ", round(time.process_time() - start, 3), "seconds.")
    # Animation.
    car_ani = animation.FuncAnimation(fig, plot_update, frames=range(1, len(state_i)),
                                      interval=100, repeat=True, blit=False)
    car_ani.save('result_demo.gif')

    plt.show()

run_sim(sim_opt)