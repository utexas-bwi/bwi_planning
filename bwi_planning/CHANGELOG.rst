^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package bwi_planning
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.0 (2014-04-23)
------------------

* Initial release to Hydro.
* Moved clasp to a separate repository.
* Fixed bugs while marking door, also added backwards compatible
  action goto to base action executor.
* Add missing dependency on gringo.
* Now print expected next state when observations don't match up.
* Added an abstract planner.
* Spring Symposium code is now working.
* Updating bwi_planning code for the ICAPS 2014 paper.
* Added ability to track planning times at each iteration.
* Added a script to run the experiment and reset it as necessary.
* Fixed heuristics in krr2014 version.
* Fixed simulation door file. corners no longer needed with the door.
* Added roslaunch unit test and missing launch run dependencies.
