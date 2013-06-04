# -*- coding: utf-8 -*- 

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import rc
import scipy.io as sio
import matplotlib as mpl
mpl.use("pgf")
pgf_with_custom_preamble = {
    "pgf.texsystem": "xelatex",
    "font.family": "sans-serif", # use serif/main font for text elements
    "text.usetex": True,    # use inline math for ticks
    "pgf.rcfonts": True,   # don't setup fonts from rc parameters
    "pgf.preamble": [
         r"\usepackage{amsmath}",
         r"\usepackage{fontspec}",
         r"\usepackage{xunicode} %Unicode extras!",
         r"\usepackage{xltxtra}  %Fixes",
         r"\setmainfont[Ligatures={Common,TeX}]{Minion Pro}",
         r"\setsansfont[Ligatures={Common,TeX}]{Myriad Pro}",
         r"\usepackage{sfmath}"
         ]
}
mpl.rcParams.update(pgf_with_custom_preamble)

torques = np.array([2.5e-3, 3e-3, 3.25e-3, 3.5e-3])

data = sio.loadmat('data.mat')
closedResponse1= np.array(data['closedResponse1'][0])
closedResponse2= np.array(data['closedResponse2'][0])
phases1= np.array(data['phases1'][0])
phases2= np.array(data['phases2'][0])

#w/ heuristic
fig = plt.figure(num = 1, figsize = (12,7))
#result
ax1 = plt.subplot(211)
ax1.spines['right'].set_visible(False)
ax1.spines['top'].set_visible(False)
ax1.xaxis.set_ticks_position('bottom')
ax1.yaxis.set_ticks_position('left')

for i in np.arange(4):
    plt.plot(closedResponse1[i][:,0],closedResponse1[i][:,1], linewidth=2, label = str(torques[i]*1e3) + ' mN-m')

#set title
plt.title('Response with Heuristic')
curpos = ax1.title.get_position()
ax1.title.set_position((curpos[0], curpos[1] + 0.025))

#set and align axis labels
plt.ylabel('roll angle [degrees]')
labelx = -0.05
ax1.yaxis.set_label_coords(labelx, 0.5)
plt.legend(bbox_to_anchor=(1.05, -0.55),loc = 3,borderaxespad=0.,frameon=False)
plt.axis([0, 10, -20, 20])

#phase relationships
ax2 = plt.subplot(212)
for i in np.arange(4):
    plt.plot(phases1[i][:,0],phases1[i][:,1], linewidth=2, label = str(torques[i]*1e3) + ' mN-m')

#set title
plt.title('Leg Phase Relationship')
curpos = ax2.title.get_position()
ax2.title.set_position((curpos[0], curpos[1] + 0.025))

#set and align axis labels
plt.xlabel('time [s]')
plt.ylabel('phase [radians]')
ax2.yaxis.set_label_coords(labelx, 0.5)
plt.axis([0, 10, 0, 2*np.pi])
ax2.spines['right'].set_visible(False)
ax2.spines['top'].set_visible(False)
ax2.xaxis.set_ticks_position('bottom')
ax2.yaxis.set_ticks_position('left')
ax2.yaxis.set_ticks([0, np.pi/2, np.pi, 3*np.pi/2, 2*np.pi])
ax2.yaxis.set_ticklabels(["$0$", r"$\frac{\pi}{2}$", r"$\pi$", r"$\frac{3\pi}{2}$", r"$2\pi$"])
for label in ax2.get_yticklabels(): 
    label.set_horizontalalignment('center') 
    label.set_position((-0.011,0))
plt.tight_layout()
plt.savefig('wheuristic.pdf', bbox_inches='tight')

#clear figures
fig = None
ax1 = None
ax2 = None

#w/o heuristic
fig = plt.figure(num = 2, figsize = (12,7))
ax1 = plt.subplot(211)
ax1.spines['right'].set_visible(False)
ax1.spines['top'].set_visible(False)
ax1.xaxis.set_ticks_position('bottom')
ax1.yaxis.set_ticks_position('left')
for i in np.arange(4):
    plt.plot(closedResponse2[i][:,0],closedResponse2[i][:,1], linewidth=2, label = str(torques[i]*1e3) + ' mN-m')

#set title
plt.title('Response without Heuristic')
curpos = ax1.title.get_position()
ax1.title.set_position((curpos[0], curpos[1] + 0.025))

#set and align axis labels
plt.ylabel('roll angle [degrees]')
labelx = -0.05
ax1.yaxis.set_label_coords(labelx, 0.5)
plt.legend(bbox_to_anchor=(1.05, -0.55),loc = 3,borderaxespad=0.,frameon=False)
plt.axis([0, 10, -20, 20])

#phase relationships
ax2 = plt.subplot(212)
ax2.spines['right'].set_visible(False)
ax2.spines['top'].set_visible(False)
ax2.xaxis.set_ticks_position('bottom')
ax2.yaxis.set_ticks_position('left')
for i in np.arange(4):
    plt.plot(phases2[i][:,0],phases2[i][:,1], linewidth=2, label = str(torques[i]*1e3) + ' mN-m')

#set title
plt.title('Leg Phase Relationship')
curpos = ax2.title.get_position()
ax2.title.set_position((curpos[0], curpos[1] + 0.025))

#set and align axis labels
plt.xlabel('time [s]')
plt.ylabel('phase [radians]')
ax2.yaxis.set_label_coords(labelx, 0.5)
plt.axis([0, 10, 0, 2*np.pi])
ax2.spines['right'].set_visible(False)
ax2.spines['top'].set_visible(False)
ax2.xaxis.set_ticks_position('bottom')
ax2.yaxis.set_ticks_position('left')
ax2.yaxis.set_ticks([0, np.pi/2, np.pi, 3*np.pi/2, 2*np.pi])
ax2.yaxis.set_ticklabels(["$0$", r"$\frac{\pi}{2}$", r"$\pi$", r"$\frac{3\pi}{2}$", r"$2\pi$"])
for label in ax2.get_yticklabels(): 
    label.set_horizontalalignment('center') 
    label.set_position((-0.011,0))
plt.tight_layout()

plt.savefig('woheuristic.pdf', bbox_inches='tight')
