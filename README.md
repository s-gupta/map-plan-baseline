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
our past work on learning policy for this task [[1](#references)].

### Method

### Visualizations
We visualize built maps and executed paths for some sample success cases. We
then show current failure modes. Visualizations are from the Habitat Gibson
validation set. 
<img src="vis/0557.png" height=200>

### References
[1] [Cognitive Mapping and Planning for Visual
Navigation](https://arxiv.org/pdf/1702.03920.pdf). IJCV (Accepted with Minor
Revisions) 2019. S. Gupta, V. Tolani, J. Davidson, S. Levine, R. Sukthankar,
and J. Malik.\
[2] [Cognitive Mapping and Planning for Visual
Navigation](https://arxiv.org/pdf/1702.03920v2.pdf). CVPR 2017. S. Gupta, J.
Davidson, S. Levine, R. Sukthankar, and J. Malik.
