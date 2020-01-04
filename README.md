# RaiderLib
Team 75 FRC Library
by Derek Geng

Includes:

  -Geometry package for non-linear position estimation on field
  
  -Path package:
  
    -Built with Cubic Hermite Splines
    
    -Abstract Path class to easily inject waypoints and headings
    
    -Able to generate velocities for each point on the path that satisfy constraints
    
  -Abstraction layer for an Adaptive Pure Pursuit Controller:
  
    -Able to produce left and right trajectories for skid-steer drive (tank drive) to follow a path
    
    -Also able to pursuit specified points by following an arc
    
    
To use as jitpack dependency, go to https://jitpack.io/#derekgeng15/RaiderLib

Resources used:

  -https://www.cubic.org/docs/hermite.htm

  -https://www.ri.cmu.edu/pub_files/pub3/coulter_r_craig_1992_1/coulter_r_craig_1992_1.pdf

  -https://www.chiefdelphi.com/t/paper-implementation-of-the-adaptive-pure-pursuit-controller/166552

  -254's presentation on motion control: https://www.youtube.com/watch?v=8319J1BEHwM
