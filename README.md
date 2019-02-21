# PerceptionAndAvoidance-ROS

The objective of this project was to perform perception using a laser range finder, and use the perceived information to avoid obstacles and navigate to a given destination using Bug2 in ROS.


PERCEPTION USING LASER RANGE FINDER

For this part, the RANSAC algorithm was implemented to determine the walls visible to the robot from the data obtained from the laser range finder. The program takes the laser scans as inputs and outputs a set of lines seen by the robot identifying the obstacles in view.
From the set of points the laser range finder gives, two random points are picked and a line drawn. The distance is found of each of the
other points from this line, and binned as inliers and outliers based on if the distance is lesser or greater than a threshold distance. This is repeated for k iterations. After k iterations, the line is picked that has the most number of inliers. Those points are dropped, and the algorithm repeated for the remaining set of points until we have lower than a threshold number of points. 

For visualization the detected lines are published on markers that can be visualized in rviz . 



OBSTACLE AVOIDANCE USING BUG2 ALGORITHM

In this part the robot starts at a fixed point and plans its path to another point. The robot navigates itself avoiding the various obstacles in its way using the standard bug2 algorithm. The robot will be in one of two states: GOAL SEEK and WALL FOLLOW. It follows the WALL by using the lines detected in the previous part to drive in parallel to it and when it dowsn't detect any walls it starts to go towards the goal following the line from the starting point to the end point.
