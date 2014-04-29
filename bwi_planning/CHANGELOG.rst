^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package bwi_planning
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.1 (2014-04-29)
------------------
* added dependency on generated service file. closes `#10 <https://github.com/utexas-bwi/bwi_planning/issues/10>`_
* modified cost_learner to have a unique build name. closes `#4 <https://github.com/utexas-bwi/bwi_planning/issues/4>`_
* Added support for YAML-CPP 0.5+.
  The new yaml-cpp API removes the "node >> outputvar;" operator, and it
  has a new way of loading documents. 
* Contributors: Piyush Khandelwal, Scott K Logan

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
