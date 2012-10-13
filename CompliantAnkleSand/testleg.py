import numpy as np
import matplotlib.pyplot as plt
import yaml
from leg import *

inputFile = open('inputs.yaml')
inputs = yaml.load(inputFile)

linkLengths = inputs['linkLengths']
leg1 = Leg(linkLengths,np.array([0, 0]),1)

plt.figure()
i = 0
footPos = []
for theta in np.linspace(0,2*np.pi,360):
	angles = leg1.calcAngles(theta)
	O, A, B, C, F = leg1.getPos()
	footPos.append(F)
	if np.mod(i,15) == 0:
		legPts = np.array([O, A, F, B, C])
		plt.plot(legPts[:,0],legPts[:,1],'k',lw = 0.1, markersize = 4, markeredgewidth = 0)
	i += 1

plt.title('Leg Kinematics')
footPts = np.array(footPos)
plt.plot(footPts[:,0],footPts[:,1])
plt.axis('equal')
x1, x2, y1,y2 = plt.axis()
plt.savefig('leg.pdf', bbox_inches='tight')
