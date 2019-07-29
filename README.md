#### General
Lidar measurements are red circles, radar measurements are blue circles with an arrow pointing in the direction of the observed angle, and estimation markers are green triangles.

#### Data file description

The main code reads in the data file line by line. The measurement data for each line gets pushed onto a measurement_pack_list. The ground truth [p_x, p_y, v_x, v_y] [px, py, vx, vy] for each line in the data file gets pushed on to ground_truth so RMSE can be calculated later from `tools.cpp`.

#### 1. Code must compile without errors with cmake and make.

Code Can complile without any error and I don't have any change on CmakeList.txt file.

#### 2. Your algorithm will be run against Dataset 1 in the simulator which is the same as "data/obj_pose-laser-radar-synthetic-input.txt" in the repository. We'll collect the positions that your algorithm outputs and compare them to ground truth data. Your px, py, vx, and vy RMSE should be less than or equal to the values [.11, .11, 0.52, 0.52].

 final result can be seen from the image below. As can be seened RMSE value is below [.11, .11, 0.52, 0.52].
![alt title][image1]


#### 3. While you may be creative with your implementation, there is a well-defined set of steps that must take place in order to successfully build a Kalman Filter. As such, your project should follow the algorithm as described in the preceding lesson.


I didn't change project structure and implement accordingly.

#### 4. Your algorithm should use the first measurements to initialize the state vectors and covariance matrices.

In `ProcessMeasurement` method and `Constructor` of `FusionEKF Class` I firstly initialize state vektors and  covariance metrices.

#### 5. Upon receiving a measurement after the first, the algorithm should predict object position to the current timestep and then update the prediction using the new measurement.
In `ProcessMeasurement` method  of `FusionEKF Class` in else part (means after first initialize) I made the program predict and update states according to data type .

#### 6. Your algorithm sets up the appropriate matrices given the type of measurement and calls the correct measurement function for a given sensor type.
Algorithm sets up the appropriate matrices given the type of measurement and calls the correct measurement function for a given sensor type.

#### 7.  Code Efficiency:
I try to avoid "code smell". For purpose of code maintain good practice with respect to calculations I avoids issue below.
Running the exact same calculation repeatedly when you can run it once, store the value and then reuse the value later.
Loops that run too many times.
Creating unnecessarily complex data structures when simpler structures work equivalently.
Unnecessary control flow checks.


[//]: # (Image References)
[image1]: img/resultScreen.JPG "Undistorted"
