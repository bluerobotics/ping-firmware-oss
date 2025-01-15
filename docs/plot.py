import os
import json
import numpy as np
import matplotlib.pyplot as plt

results = []
for root, dirs, files in os.walk("."):
    for file in files:
        if file.endswith(".json"):
            results.append(os.path.join(root, file))

for result in results:
    with open(result, 'r') as f:
        data = json.load(f)
        name = data['name']
        times_s = data['times_s']
        samples = data['samples']

        # HAL_GPIO_WritePin (SET and RESET around 597 ns)
        hal_interference_s = 597.62e-9
        # Remove HAL overhead
        times_s = [ x + hal_interference_s for x in times_s ]

        x = np.array(samples)
        y = np.array(times_s)
        z = np.polyfit(x, y, 1)
        p = np.poly1d(z)

        time_per_sample_ns = 1e9 * z[0]

        plt.clf()
        plt.plot(samples, times_s, 'ro', label='Measured Data')
        plt.plot(x, p(x), 'g-', label=f'Linear Fit: y = {z[0]:.2e}x + {z[1]:.2e}')
        plt.title(f"Performance [{name}] - AVG Time per sample: {time_per_sample_ns:.3f} ns")
        plt.xlabel('N')
        plt.ylabel('Time (s)')
        plt.grid(alpha=0.5)
        plt.legend()
        plt.gcf().set_size_inches(10, 6)
        plt.savefig(f"./{result.split('.')[1]}.png")
