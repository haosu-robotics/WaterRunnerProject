Water Contact Modeling
======================

This folder contains a document and a powerpoint presentation that outline the mathematics behind the water contact modeling. The approach in the [literature][1] integrates the pressure over the foot pad area to obtain the force on a foot. When implemented in Simulink, this approach is very slow as Simulink does not have any built in functions that perform integration over variables other than time. Therefore, in order to speed up this calculation, a change of variables is performed in order to transform the integration over foot pad area, into an integration over time. This integration can be performed in Simulink very efficiently.

[1]: http://ieeexplore.ieee.org/xpls/abs_all.jsp?arnumber=4542866       "IEEE Xplore"
