import numpy as np
import matplotlib.pyplot as plt

def read_jam_factor(path):
    last_time = 0
    data = []
    with open(path) as f:
        line = f.readline()
        while line:
            s = line.split(' ')
            s = [float(x) for x in s]
            t = s[0]

            if int(t) >= last_time + 30:
                data.append((t,
                    s[4] / s[1], 
                    s[5] / s[2], 
                    s[6] / s[3]))
                print(path)
                print(sum(s[4:7]) / sum(s[1:4]))
                last_time = int(t)
            line = f.readline()
    return data

data_gamma = read_jam_factor('statistics.gamma.log')
data_simple = read_jam_factor('statistics.ttc.log')

avg_total = 0
avg_count = 0
for x in data_gamma:
    avg_total += x[1] + x[2] + x[3]

fig, axs = plt.subplots(figsize=(17.5, 8))
axs.set_xlabel('Simulation Time (s)', fontsize=34)
axs.set_ylabel('Congestion Factor', fontsize=34)
plt.xlim(0, 1200)
plt.ylim(-0.01, 1.19)
plt.rc('legend',fontsize=26)
axs.tick_params(axis='x', labelsize=32)
axs.tick_params(axis='y', labelsize=32)

# GAMMA Car Jam Factor.
axs.plot(
        [x[0] for x in data_gamma], 
        [x[1] for x in data_gamma],
        label='Car (C-GAMMA)',
        color='red',
        linestyle='solid',
        linewidth=3)

# TTC Car Jam Factor.
axs.plot(
        [x[0] for x in data_simple], 
        [x[1] for x in data_simple],
        label='Car (TTC)',
        color='red',
        linestyle=(0, (5, 5)),
        linewidth=3)

# GAMMA Bike Jam Factor.
axs.plot(
        [x[0] for x in data_gamma], 
        [x[2] for x in data_gamma],
        label='Bike (C-GAMMA)',
        color='green',
        linestyle='solid',
        linewidth=3)

# TTC Bike Jam Factor.
axs.plot(
        [x[0] for x in data_simple], 
        [x[2] for x in data_simple],
        label='Bike (TTC)',
        color='green',
        linestyle=(0, (5, 5)),
        linewidth=3)


# GAMMA Ped. Jam Factor.
axs.plot(
        [x[0] for x in data_gamma], 
        [x[3] for x in data_gamma],
        label='Pedestrian (C-GAMMA)',
        color='blue',
        linestyle='solid',
        linewidth=3)


# TTC Ped. Jam Factor.
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
