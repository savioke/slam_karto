^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package slam_karto
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Removed a bunch of old launch files
* SlamKarto still not publishing loop closures to match karto optimization...
* Added default launch file and publishing of response strength in loop closure
* Launch files
* calibration working
* Plugins working
* Plugins working, not getting enough loop closures
* Fixed formatting
* Introduced base class for solvers and plugin loading of solver classes
* Pluginlib support for loading SLAM solvers (SPA only so far)
  Conflicts:
  src/slam_karto.cpp
* Exposes open karto parameters in the ROS wrapper
* Contributors: liz-murphy

0.7.1 (2014-06-17)
------------------
* build updates for sba, fix install
* Contributors: Michael Ferguson

0.7.0 (2014-06-15)
------------------
* First release in a very, very long time.
* Catkinized, updated to work with catkinized open_karto and sba
* Contributors: Jon Binney, Michael Ferguson
