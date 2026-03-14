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
* bazel (7.x+)
* Eigen (3.4.0, fetched by Bazel)
* Poco (1.11+)
* libconfig (1.5+)
* lua (5.1)
* tbb
* GNU Scientific Library (GSL)
* Point Cloud Library (PCL) (1.14+)

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

or:
```
$ docker build -t ian:l3
$ docker run ian:l3
```
