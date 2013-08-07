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
    "pgf.rcfonts": True,   
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

heights = np.array([5e-3, 6e-3, 7e-3, 8e-3])

data = sio.loadmat('data1.mat')
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

for i in np.arange(heights.shape[0]):
    plt.plot(closedResponse1[i][:,0],closedResponse1[i][:,1], linewidth=2, label = str(heights[i]*1e3) + ' mm')

#set title
plt.title('Roll Angle Response')
curpos = ax1.title.get_position()
ax1.title.set_position((curpos[0], curpos[1] + 0.025))

#set and align axis labels
plt.ylabel('roll angle [deg]')
labelx = -0.06
ax1.yaxis.set_label_coords(labelx, 0.5)
plt.legend(bbox_to_anchor=(1.05, -0.75),loc = 3,borderaxespad=0.,frameon=False,title='Amplitude')
plt.axis([0, 8, -30, 10])

#phase relationships
ax2 = plt.subplot(212)
for i in np.arange(heights.shape[0]):
    plt.plot(phases1[i][:,0],phases1[i][:,1], linewidth=2, label = str(heights[i]*1e3) + ' mm')

#set title
plt.title('Leg Phase Difference')
curpos = ax2.title.get_position()
ax2.title.set_position((curpos[0], curpos[1] + 0.025))

#set and align axis labels
plt.xlabel('time [s]')
plt.ylabel('phase difference [rad]')
ax2.yaxis.set_label_coords(labelx, 0.5)
plt.axis([0, 8, 0, 2*np.pi])
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
for i in np.arange(heights.shape[0]):
    plt.plot(closedResponse2[i][:,0],closedResponse2[i][:,1], linewidth=2, label = str(heights[i]*1e3) + ' mm')

#set title
plt.title('Roll Angle Response')
curpos = ax1.title.get_position()
ax1.title.set_position((curpos[0], curpos[1] + 0.025))

#set and align axis labels
plt.ylabel('roll angle [deg]')
ax1.yaxis.set_label_coords(labelx, 0.5)
plt.legend(bbox_to_anchor=(1.05, -0.75),loc = 3,borderaxespad=0.,frameon=False,title='Amplitude')
plt.axis([0, 8, -30, 10])

#phase relationships
ax2 = plt.subplot(212)
ax2.spines['right'].set_visible(False)
ax2.spines['top'].set_visible(False)
ax2.xaxis.set_ticks_position('bottom')
ax2.yaxis.set_ticks_position('left')
for i in np.arange(heights.shape[0]):
    plt.plot(phases2[i][:,0],phases2[i][:,1], linewidth=2, label = str(heights[i]*1e3) + ' mm')

#set title
plt.title('Leg Phase Difference')
curpos = ax2.title.get_position()
ax2.title.set_position((curpos[0], curpos[1] + 0.025))

#set and align axis labels
plt.xlabel('time [s]')
plt.ylabel('phase difference [rad]', multialignment = 'center')
ax2.yaxis.set_label_coords(labelx, 0.5)
plt.axis([0, 8, 0, 2*np.pi])
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
