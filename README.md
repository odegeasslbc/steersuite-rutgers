# Computer Graphics assignments and projects based on SteerSuit

## Curves

Implemented Hermite and Catmull Rom splines 
 
## Collisions Detection

Implemented the GJK and EPA algorithms for polygonal obstacles in 2D. 

The input will be 3D points that lie on the XZ plane, ie. Y = 0.

The output will be whether there is a collision or not.

Finally, if there is a collision, the EPA algorithm will be called to find the penetration depth and
penetration vector for the collision.

I also included a concav decomposition function that devide a concav polygon into convexs components,
that make it able to run GJK_EPA on complex polygons.

## Social Forces

Implement different social forces algorithms which will be applied to agents during crowd motion simulations. 

###A. Individual Behaviors
      a. Pursue and Evade
      b. Wall Follower (The Maze Solver)
      c. Growing Spiral
      d. Unaligned Collision Avoidance
###B. Group Behaviors
      a. Crow Follower
      b. Leader Following
      c. Queueing
###C. Highlighted
      a. Full Flocks Implementation (cohesion, separations and alignment) 
      b. Deformable Agents
      c. Elliptical Collision
      d. Area Deadlock Resolution 
