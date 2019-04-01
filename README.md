# pso
Particle Swarm Optimization for go

Based on slides from https://www.mii.lt/zilinskas/uploads/Heuristic%20Algorithms/Lectures/Lect4/PSO2.pdf  Please use that as reference.  

Added some vales like minstart and maxstart.  Lets say you are wanting to optimize the meta values of a neural network.  You know that you don't want the learning rate anywhere near 1 or even greater than one.  You can set it it minxstart to .001 and maxxstart to .01. So it doesn't go crazy trying to find the optimum values.  

Most of these functions are not thread safe.  I tried to make the AsyncUpdate and the IndvSyncUpdate methods thread safe, but they are not tested.  

If you know a better way to allow the users to parallelize this then please let me know.

TODO:

1) Add a custom function mode.
2) Add tests functions.


MAYBEDO:

Add a SwarmInt