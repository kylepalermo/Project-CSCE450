Kyle Palermo 519004829 kylepalermo@tamu.edu

I have completed tasks 1-4.

A reasonable range for the compliance parameter is about [0, 0.1], although if very
stretchy behavior is desired it can be higher. The cloth becomes more active 
and folds on itself more often below 0, the cloth starts to fold itself completely 
below -0.5, and the cloth stretches itself out infinitely below -0.9. A reasonabale 
range for the damping parameter is about [1e-4, 1e-5]. Above 1e-3, the cloth moves 
very slowly, as if through a thick medium, and above 2.5e-2, the cloth simulation 
collapses immediately. The cloth folds in on itself frequently below 1e-6, it swings 
perpetually at 0, it bounces and gains energy below -1e-5, and it stretches 
infinitely below -1e-4. A reasonable range for the time step is [1e-3, 5e-4], 
although this can be lower with sufficient resources or if it is not simulated in 
real time. Above 1e-3, the cloth becomes stretched much more than normal, and below 
5e-4, the simulation is very slow. On my computer in release mode, 100x100 runs at 
a speed that appears realistic, 1000x1000 runs with some visible movement, and 
4000x4000 crashes before it can allocate enough memory.