# unc-f1tenth

UNC team for F1/10 autonomous racing competition

1. Run `catkin_make` in the root folder of the repository.
2. Open a few terminals in the root folder of the repository.
3. In each terminal run `. devel/setup.bash`
4. If you want to use the UnrealF1Tenth simulation environment, run `roslaunch rosbridge_server rosbridge_tcp.launch bson_only_mode:=True`. If you are using a physical F1Tenth car, or your own simulator, ignore this step.
5. In one of the terminals, run `rosrun emergency_stop estop_filter.py`
6. In another terminal, run a controller:
    * `rosrun voronoi main` or
    * `python2.7 src/disparity_extender/src/disparity_extender_time_trial.py`
7. To visualize the output of the voronoi controller, run `rosrun rviz rviz src/voronoi/voronoi.rviz`
