import numpy as np
import matplotlib.pyplot as plt

INTERVAL = 30
WINDOW = 5

def read_avg_speed(path):
    data = []
    with open(path) as f:
        last_time = 0
        line = f.readline()
        window_total_speed = [0, 0, 0]
        window_num = 0

        global_total_speed = 0
        global_num = 0

        while line:
            s = line.split(' ')
            s = [float(x) for x in s]
            t = s[0]
            
            if int(t) >= last_time + INTERVAL - WINDOW and int(t) < last_time + INTERVAL + WINDOW:
                window_total_speed[0] += s[7]
                window_total_speed[1] += s[8]
                window_total_speed[2] += s[9]
                window_num += 1
            elif int(t) >= last_time + INTERVAL + WINDOW:
                data.append((
                    last_time + INTERVAL, 
                    window_total_speed[0] / window_num,
                    window_total_speed[1] / window_num,
                    window_total_speed[2] / window_num))
                window_total_speed = [0, 0, 0]
                window_num = 0
                last_time = last_time + INTERVAL
            line = f.readline()

        print(path)
        
    return data

data_gamma = read_avg_speed('statistics.gamma.log')
data_simple = read_avg_speed('statistics.ttc.log')

fig, axs = plt.subplots(figsize=(17.5, 8))
axs.set_xlabel('Simulation Time (s)', fontsize=34)
axs.set_ylabel('Average Speed (m/s)', fontsize=34)
plt.xlim(0, 1200)
plt.ylim(0, 6)
plt.rc('legend',fontsize=26)
axs.tick_params(axis='x', labelsize=32)
axs.tick_params(axis='y', labelsize=32)

# GAMMA Car.
axs.plot(
        [x[0] for x in data_gamma], 
        [x[1] for x in data_gamma],
        label='Car (C-GAMMA)',
        color='red',
        linestyle='solid',
        linewidth=3)

# TTC Car.
axs.plot(
        [x[0] for x in data_simple], 
        [x[1] for x in data_simple],
        label='Car (TTC)',
        color='red',
        linestyle=(0, (5, 5)),
        linewidth=3)

# GAMMA Bike.
axs.plot(
        [x[0] for x in data_gamma], 
        [x[2] for x in data_gamma],
        label='Bike (C-GAMMA)',
        color='green',
        linestyle='solid',
        linewidth=3)

# TTC Bike.
axs.plot(
        [x[0] for x in data_simple], 
        [x[2] for x in data_simple],
        label='Bike (TTC)',
        color='green',
        linestyle=(0, (5, 5)),
        linewidth=3)

# GAMMA Pedestrian.
axs.plot(
        [x[0] for x in data_gamma], 
        [x[3] for x in data_gamma],
        label='Pedestrian (C-GAMMA)',
        color='blue',
        linestyle='solid',
        linewidth=3)

# TTC Pededestrian.
axs.plot(
        [x[0] for x in data_simple], 
        [x[3] for x in data_simple],
        label='Pedestrian (TTC)',
        color='blue',
        linestyle=(0, (5, 5)),
        linewidth=3)

axs.legend(loc="upper right", ncol=3)
fig.tight_layout()
plt.show()
