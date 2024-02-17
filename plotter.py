import pandas as pd
import matplotlib.pyplot as plt

# Les inn CSV-filen
df = pd.read_csv('/home/jorgen/data_test_ntnu/python_data/data_export_2024-02-17-14-41-55.csv', delimiter=',')  # Antar at tab er delimiter


time_list = df['time'].tolist()

u1_list = df['u1'].tolist()

u2_list = df['u2'].tolist()

u3_list = df['u3'].tolist()

lin_vel_list = df['lin_vel'].tolist()

angVel_list = df['angVel'].tolist()


# Plot hver av datakolonnene mot tiden
plt.figure(figsize=(10, 8))

# Plot lin_vel mot time
plt.subplot(3, 1, 1)  # 3 rader, 1 kolonne, første plot
plt.plot(time_list, lin_vel_list, label='Lineær Hastighet')
plt.xlabel('Tid (s)')
plt.ylabel('Lineær Hastighet')
plt.legend()

# Plot angVel mot time
plt.subplot(3, 1, 2)  # 3 rader, 1 kolonne, andre plot
plt.plot(time_list, angVel_list, label='Vinkelhastighet', color='orange')
plt.xlabel('Tid (s)')
plt.ylabel('Vinkelhastighet')
plt.legend()

# Plot u1, u2, og u3 mot time i samme plot for sammenligning
plt.subplot(3, 1, 3)  # 3 rader, 1 kolonne, tredje plot
plt.plot(time_list, u1_list, label='u1')
plt.plot(time_list, u2_list, label='u2')
plt.plot(time_list, u3_list, label='u3')
plt.xlabel('Tid (s)')
plt.ylabel('U-verdier')
plt.legend()

# Vis plot
plt.tight_layout()  # Justerer subplot slik at de ikke overlapper

plt.show()

