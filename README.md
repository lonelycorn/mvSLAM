# mvSLAM
beginner's monocular visual SLAM

=== Dependency ===

pcl = 1.7

vtk = 6.2

gtsam >= 3.2.1

opencv >= 3.0

eigen >= 3.0

boost >= 1.54

pthread


=== TODO ===

drop dependency of opencv

=== Known Issues ===

May crash on 32-bit machines.
This is because Eigen needs 16-byte aligned allocator, which is the default behavior of 64-bit machines, but not guaranteed for 32-bit machines.
There is an official wiki page describing this issue and providing solutions:
    http://eigen.tuxfamily.org/dox-devel/group__TopicStlContainers.html


