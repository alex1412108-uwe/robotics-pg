# robotics-pg
team LDCA repo for robotics pg coursework

Current status (by Dave, 7/3/15):
This seems to be working now...
CHANGES:
- Added in a factor to the probability related to how turned the particle is
- - This makes particles pointing in the right direction more likely
- Changed the calculation of the angle a bit
TODO:
- Optimise the hell out of it
- Check the modify map function works... it doesn't seem to use the modified map?
- - Haven't checked, but are we passing in ModifiedMap to the A*?
- Find out appropriate number of points to skip in A* route 
- - (I'm doing 3 because it was a nice number)
- Code for checking particles are within boundary
- - it also doesn't work too well if the robot's too close to a wall




To run:
Use EXAMPLE5localise.m
-> This initialises the map, inital bot position and then calls localise.m
----> localise.m is the file we've been playing with. 