^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package slam_karto
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Merge pull request `#4 <https://github.com/savioke/slam_karto/issues/4>`_ from liz-murphy/no_loop_response
  Remove loop closure response publisher (removed from open karto too)
* Remove loop closure response publisher (removed from open karto too)
* Merge pull request `#3 <https://github.com/savioke/slam_karto/issues/3>`_ from liz-murphy/cosmetic
  Cosmetic
* Tidied up ROS_INFO messages
* Maps aligned with orientation of first scan
* Cosmetic changes to map creation so its aligned, think transforms are wrong
* Contributors: Jonathan Binney, Stephan, liz-murphy

100.7.4 (2014-10-24)
--------------------
* Install karto_spa_plugin
* Contributors: Jon Binney

100.7.3 (2014-10-24)
--------------------
* Install pluginlib xml file
* Contributors: Jonathan Binney

100.7.2 (2014-10-24)
--------------------
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
