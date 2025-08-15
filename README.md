# ROS2Logger

## This is my custom ROS2 Logger that does the following for you:
1. Provides custom data types for logging an sequence of values (and even a data type for a sequence of vector values) via appending like a list
2. Provides shutdown hooks so that when your ROS2 Node terminates via exit(0)/raised Error/etc, it will automatically pull variables of these data types from your node
3. It will then assign them columns in a .log file ordered by a desired odering integer value you provide at the point of instantiation or alphabetically to break ties / ambiguous order in the vector case

*Note that for my purposes the logger currently stipulates time,x,y,z,yaw as necessary logged variables to ensure I have the bare minimum I need from my quadrotor experiments. Users may alter this.*

5. It then generates a '/data_analysis' subdirectory within the directory of the ROS2 node that called the Logger
6. It also automatically populates this directory with a DataAnalysis.ipynb equipped with tools I commonly use for my work (RMSE, Plotting, overleaf/latek-compatible pdf exports for the plots, etc) as well as a subdirectory '/data_analysis/log_files' where the log files are saved

## How to use:
1. Create a ROS2 workspace with a src/ directory
3. Git clone this remote repository into the src/ of your ROS2 workspace
```
git clone git@github.com:evannsm/ROS2Logger.git
```
3. Go to the root of your ROS2 workspace and build (symbolic links preferrable)
```
colcon build --symlink-install
```
4. Use the test_logger package for a quick and easy example of this system in action, and learn to operate this Logger package!
