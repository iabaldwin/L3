L3
==

Long-term Localisation with LIDAR.

See:
*   Baldwin, I.A, Newman, P. (2011) Road vehicle localization with 2D push-broom
    lidar and 3D priors.  In Proc. IEEE International Conference on Robotics and
    Automation (ICRA), St. Paul, MN. 

Preview
-------
![L3](/media/screenshots/4.png)

See: [Overview](https://vimeo.com/81037811)

Requirements
------------
* bazel (0.18.0)
* Eigen (3.3.4)
* Poco (1.8.0)
* libconfig (1.5.0)
* lua (5.1.0)
* tbb (2017-U7-8)
* GNU Scientific Library (GSL) (2.4)
* Point Cloud Library (PCL) (1.8.1)

Build
-----
```
$ bazel build -c opt "..."
```

Run 
-----
```
$ bazel run -c opt //app:app -- --path/to/dataset
```
