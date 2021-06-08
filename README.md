# unc-f1tenth

UNC team for F1/10 autonomous racing competition

1. Run `catkin_make` in the root folder of the repository. The build may fail due to some dependecies. Read the errors and install the missing packages on your system.
2. Open a few terminals in the root folder of the repository.
3. In each terminal run `. devel/setup.bash`
4. If you want to use the UnrealF1Tenth simulation environment, run `roslaunch rosbridge_server rosbridge_tcp.launch bson_only_mode:=True`. This local version of rosbridge works with the [ROSIntegration plugin for UE4](https://github.com/code-iai/ROSIntegration) used in UnrealF1Tenth. If you are using a physical F1Tenth car, or your own simulator, ignore this step.
5. In one of the terminals, run `rosrun emergency_stop estop_filter.py`. 
6. Run a controller:
    * Start the voronoi algorithm with `rosrun voronoi main` and stop it with Ctrl+C.
    * Run `rosrun emergency_stop estop_sender.py` in one terminal, and start the disparity algorithm with `python2.7 src/disparity_extender/src/disparity_extender_time_trial.py` in another terminal. To stop the car, hit the spacebar while in the estop_sender terminal. After an emergency stop, you need to re-run `rosrun emergency_stop estop_filter.py` to be able to run the car again.
7. To visualize the output of the voronoi controller, run `rosrun rviz rviz src/voronoi/voronoi.rviz`
