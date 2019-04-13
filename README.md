## Classical Mapping and Planning Baseline for Habitat Challenge
[Saurabh Gupta](http://saurabhg.web.illinois.edu/) \
Facebook AI Research

### Abstract
This baseline implements a basic map building and path planning algorithm for
the task of reaching a specified point under known odometry with depth images
as input. Depth images are back-projected to obtain a 3D point cloud. Points in
the 3D point cloud that lie within the robot height are projected on the 2D
plane to obtain an occupancy map. This occupancy map is dilated by the radius
of the robot to obtain a traversability map. Paths are planned on this
traversability map by computing the geodesic distance to the goal using
fast-marching method, and greedily picking actions that minimize this geodesic
distance. A very similar implementation was used as a baseline in
our past work on learning policy for this task ([1])[./].

### Method
[1] Cognitive Mapping and Planning for Visual Navigation. S. Gupta, V. Tolani, J. Davidson, S. Levine, R. Sukthankar, and J. Malik.
