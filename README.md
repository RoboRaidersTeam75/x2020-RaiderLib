# RaiderLib
Team 75 FRC Library
by Derek Geng

Includes:

  -Geometry package for basic linear positioning on field
  
  -Path package:
  
    -Built with Cubic Hermite Splines(https://www.cubic.org/docs/hermite.htm)
    
    -Abstract Path class to easily inject waypoints and headings
    
    -Able to generate velocities for each point on the path that satisfy constraints
    
  -Abstraction layer for an Adaptive Pure Pursuit Controller:
  
    -Able to produce left and right trajectories for skid-steer drive (tank drive) to follow a path
    
    -Also able to pursuit specified points by following an arc
    
    -Thanks to Team 1712 for their excellent whitepaper and 254's presentation on motion control
    
