L3
==

Long-term Localisation with LIDAR.

See:
*   Baldwin, I.A, Newman, P. (2011) Road vehicle localization with 2D push-broom
    lidar and 3D priors.  In Proc. IEEE International Conference on Robotics and
    Automation (ICRA), St. Paul, MN. 

Requirements
------------

* GNU Scientific Library (GSL)
* Point Cloud Library (PCL)


Build
-----
ccmake -DPCL_DIR={PCL_DIR} ...

Core
----
* Core components, point-clouds, projection, swathe-building
* Render 

Renderer
--------
* GLV-based renderer
