.. Follow REP-0132: https://www.ros.org/reps/rep-0132.html
..
.. Every pull request adds its entry under Forthcoming. A release pull request
.. renames Forthcoming to the version being released and stamps the date.
..
.. This file begins at 1.2.0. Releases before it kept no changelog, and the
.. rest of what 1.2.0 carried is not recorded here either; the entry below
.. covers only the change that reopened it.

^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package automatepro_bringup
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.2.0 (2026-09-11)
------------------
* ``core_driver.launch.py`` launches ``automatepro_cam1_node``,
  ``automatepro_cam2_node`` and ``automatepro_driver_manager`` with ``respawn``,
  so launch starts them again when their process exits. Nothing else in the
  launch file respawns.
* The camera nodes restart 30 seconds after exiting. The camera driver refuses
  to configure and exits when no GMSL2 serializer answers on the i2c bus, so a
  camera attached after boot previously needed an operator to restart the whole
  stack. With no camera attached the node exits on every attempt, and repeated
  start-and-exit cycles in the journal are expected rather than a fault. The
  delay is sized against someone attaching a camera, not against the refusal,
  which takes about two seconds.
* The driver manager restarts 10 seconds after exiting, sooner because nothing
  external has to change before it can start and because the camera drivers
  have no GMSL2 recovery while it is down.
* Restarts are suppressed once shutdown begins, so a stopping stack is never
  held open by a node coming back.
