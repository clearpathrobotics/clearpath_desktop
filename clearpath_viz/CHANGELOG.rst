^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package clearpath_viz
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.2.1 (2026-06-25)
------------------
* [Humble] Added nav2_rviz_plugins exec dep and changed to clearpath_description… (`#30 <https://github.com/clearpathrobotics/clearpath_desktop/issues/30>`_)
  * Added nav2_rviz_plugins exec dep and changed to clearpath_description to include all meshes. (`#29 <https://github.com/clearpathrobotics/clearpath_desktop/issues/29>`_)
  (cherry picked from commit af153e38f69c5f60d8a46feff7c51f667cd31c5d)
  # Conflicts:
  #	clearpath_viz/package.xml
  * Fixed conflicts.
  ---------
  Co-authored-by: Tony Baltovski <tbaltovski@clearpathrobotics.com>
* Contributors: mergify[bot]

1.2.0 (2025-03-14)
------------------
* Feature: MoveIt Kinematics (`#20 <https://github.com/clearpathrobotics/clearpath_desktop/issues/20>`_)
  * Use kinematics file to generate parameters for markers
* Fix: handle empty namespace (`#19 <https://github.com/clearpathrobotics/clearpath_desktop/issues/19>`_)
* Contributors: luis-camero

1.0.0 (2024-11-25)
------------------
* Added minimum version.
* Contributors: Tony Baltovski

0.3.0 (2024-09-19)
------------------
* Undo changes to view_robot
* Added kinematics to RViz parameters
* Added MoveIt visualization launch file
* MoveIt rviz config
* Remappings for moveit
* Contributors: Luis Camero

0.1.2 (2023-10-13)
------------------
* rqt_robot_monitor dep
* View diagnostics
  Added license
* Pass setup path to clearpath_config
* Contributors: Luis Camero, Roni Kreinin

0.1.1 (2023-10-05)
------------------

0.1.0 (2023-08-25)
------------------
* Update scan topic to match other packages
* Contributors: Hilary Luo

* Update scan topic to match other packages
* Contributors: Hilary Luo

0.0.2 (2023-07-06)
------------------
* Added generator to launch
* Added live config to view_model
* Contributors: Luis Camero

0.0.1 (2023-07-05)
------------------
* Removed global namespace for nav2.rviz
* Added view_navigation launch file
* Updated description generator naming
  Updated robot.rviz
* Added clearpath live
* [clearpath_viz] Renamed robot_model to platform_model.
* [clearpath_viz] Removed all forward slashes on topics.
* [clearpath_viz] Added namespace and sim_time support.  Removed per platform rviz configs.
* Initial commit.
* Contributors: Luis Camero, Roni Kreinin, Tony Baltovski
