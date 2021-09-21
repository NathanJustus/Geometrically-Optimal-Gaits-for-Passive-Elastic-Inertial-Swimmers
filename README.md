# Geometrically-Optimal-Gaits-for-Passive-Elastic-Inertial-Swimmers
Code that I used to run simulations for my inertial swimmer paper, which discussed optimal motions for swimming systems with passive dynamics

General process is as follows
1) Define geometric system in sysplotter
2) Export system inertial structure from sysplotter using GUI
3) Save as .mat file
4) Use 'fitMassMatrixData.m' to generate another .mat file holding interpolant functions for computationally heavy matrix calculations
5) Use 'automateOptimization.m' to optimize fourier parameter gait using interpolant functions
6) Use 'drawRealPaths' structure to plot optimal gait on CCF
