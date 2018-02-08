from quad import*
from dynamics import*
import numpy as np

ardrone = quad()
voltaje = np.array([7.5,7.5,7.5,7.5]).reshape(-1,1)
ardrone.print_state()
for a in range(0,100):
	dynamics(voltaje,ardrone)
	ardrone.print_state()

