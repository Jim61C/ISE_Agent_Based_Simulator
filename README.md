An Agent Based Simulator for Vessel Trajectory Modelling
* Author: Xing Yifan <xingyifan@u.nus.edu>, Giulia Pedrielli <giulia.pedrielli.85@gmail.com> and Ng Szu Hui <isensh@nus.edu.sg>
* This project contains an agent based simulator emulating the vessel movements at Singapore Srait, where each simulated vessel follows the behavior pattern extracted with a certain probability. Here, stochasticity is introduced by giving a probability to each vessel to follow a different pattern each time. The vessels are modelled as individual agents and they interact with each other based on the vessel interaction model defined by the minimum maneuvering distance matrix from the [python learning module](https://gitlab.com/Jim61C/Vessel-Trajectory-Modeller). **Please cite the paper listed at the end if you use code from this repository**

* Input files needed:
  * Put under SNATII/trajectory_data/ (output from the python learning module, refer to the [REMDME.md](https://gitlab.com/Jim61C/Vessel-Trajectory-Modeller/blob/master/README.md) for more detailed information)
  		* endpoints_to_protocol_trajectories.csv
  		    * A dictionary of [Endpoint: Possible trajectories that start from this endpoint]
    	* protocol_trajectories_with_cluster_size.csv
    	    * A list of trajectory patterns learned stored with their corresponding width (pattern size)
    	* vessel_speed_to_distance.csv
    	    * A dictionary of [A pair of vessels' maneuvering relative speed: A pair of vessels' maneuvering distance]
    	* mmsi_list.csv
    	    * The list of MMSI identifier (unique indentifier) for all the vessels involved in the learning procedure
  * Put under SNATII/
    	* TestRun1.txt
    	* TestRun2.txt
    	* TestRun3.txt
    	* ...
    	* The TestRun{i}.txt files are the same cases used in the numerical section in the paper listed at the end, where each line of the TestRun{i}.txt contains the original information for each vessel, including latitude, longitude, original course, MMSI id, etc.

* Loading Patterns Modules:
  * TrajectoryLoader.cpp
  * It loads the patterns, endpoints and vessel min distance dictionary from SNATII/trajectory_data/
  
* Pattern based mover:
  * PathMoverPatternBased.cpp
  * It includes all the logic for the vessel movements accroding to the pattern learned
  
* Main function:
  * SNAT.cpp
  * It loads the TestRun{i}.txt into different vessels and initiates the start of the entire ABM procedure.

* Instruction for Running:
  * Open SNAT.sln project
  * Choose Debug mode and build accordingly
  * Input the test id, e.g., 1, 2 or 3 etc, TestRun{i}.txt will be read, i is the test id
  * Vessel Movement results is found in TargetHistory{i}.txt and could be fed to the visualizer directly
        * It contains the log of the locations, course, trajectory pattern followed, on pattern position, etc, of all the vessels in test run
    	* Output format is one log result per clock tick on each line in the txt file
    	* In the format of the following:
      		* {patternId} {onPattternPosition/trajectoryLength} {longitude} {latitude} {course} {runtime clocktick count}

<br><br><br>
* Graphical Interface - Vessel Trajectory Visualiser
  * A Google Map Web Interface based visualiser for simulation of vessel trajectories in the Singapore Strait
  
  * Heat Map toggling
  
  * Trajectory plotting and tracking
  
  * Setup:
      * Navigate to ./SNAT/Visualiser/
      * Open index.html and you are ready to go
  * Usage:
      * Upload any .csv files in the output format of cleaned data from the python learning module for overall trajectory plotting, tracking and heatmap toggling
            * For example, choose any of the {IMO}.txt file from [{vessel type}/cleaneData/](https://gitlab.com/Jim61C/Vessel-Trajectory-Modeller/tree/master/tankers/cleanedData)
      * Upload any .txt files from the ABM output for vessel movements under Agent Based Modelling (for example, click 'upload' from the visualser and choose 'TargetHistory1.txt' from ./SNAT/SNAT)
      * You can PausePlotting and RestartPlotting anytime
      * To restart plotting, click on Clear Trajectory and RestartPlotting
          * The slider bar is used to adjust the plotting speed (plotting time interval, combined with the actual vessel speed)
          * Under Functionality Dropdown menu, you can
                * Place Origin and End pins
                * Calculate the distance between the pins



- Please cite this paper if you use this code
  ```
  Pedrielli, G., Xing, Y., Peh, J.H., and Ng, S.H. A Real Time Simulation Optimization Framework for Vessel Collision avoidance and the case of Singapore Strait. Working paper, 2016.
  ```