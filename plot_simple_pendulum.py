import matplotlib.pyplot as plt
import numpy as np

time,theta,theta_dot = np.loadtxt('Simulation_result.txt', delimiter=',', unpack=True)
f = plt.figure(1)
plt.plot(time,theta, 'o-',label="Theta")
plt.plot(time,theta_dot,'r--',label="Theta_Dot")
plt.xlabel('TIME (S)')
plt.ylabel('THETA/THETA_DOT')
plt.title('Simulation Results')
plt.legend()
g = plt.figure(2)
plt.plot(theta,theta_dot, 'g-',label="Theta Vs Theta Dot ")
plt.xlabel('THETA')
plt.ylabel('THETA_DOT')
plt.title('Simulation Results')
plt.legend()
plt.show()
