##Project : Path planning

**Steps to completed **

1. Trajectory planner that ensures  car stays in a lane
2. Respect speed limit
3. Avoid exceeding the max Acceleration and Jerk
4. Path planning using state machine
4. Avoiding collision
5. Navigate around other vehicles ie lane change


#Rubric points

#1. The car can drive at least 4.32 miles without incident
    Has been tested and verified

#2. The car drives according to speed limit
    The speed limit requirement is 50 MPH. In the simulator, each trajectory
    point corresponds to a time interval of 20ms. So, spaced out trajectory
    points make the car go faster and closer trajectory points make the car
    go slower.

   the function path::get_step function located in path.h line number 336 takes
   care of this functionality.

   While calculating the next step, the euclidian distance between the previous
   and next step is selected such that the velocity does not go beyond the
   reference velocity. Also, takes care that the acceleration does not exceed
   max acceleration by clamping the values to less than the  max allowed acceleration

#3 The car does not exceed a total acceleration of 10 m/s^2 and a jerk of 10 m/s^3.
   Acceleration is the rate of change of velocity.Jerk is the rate of change of
   acceleration.in the path::get_step function.Since the time step interval is 20ms,
   it was found that the acceleration of 0.0022 does not result exceeding the
   total acceleration or maximum jerk


#4 The car must not come into contact with any of the other cars on the road

   This is taken care of by a state machine. THe state machine has the following
   states
##State machine design

 KEEP_GOING : Keep to the same lane
 PREPARE_FOR_LANE_CHANGE : Prepare to switch lanes
 TAIL_VEHICLE : tail behind another vehicle, if lane switch is not possible
 GO_LEFT: Switch to the left lane
 GO_RIGHT : Switch to the right lane

 The state machine is explained in this model.
 [image1]: ./images/statemachine.jpg

Video of the simulator can be found at the below link
youtu.be/b5252Db4WuY

1) Every state looks around the surroundings of the car and updates the states based on the sensor data. Each object in the current lane or left or right lane is tracked. Cars which are a certain distance away are checked only, instead of all cars in the current lane.(path.h : 168)

2) Costs
  Cost calculations are based on
  1) Speed differece between vehicle and object speed
  2) speed differece between ref velocity and object speed
  3) distance between vehicle and object

Costs are calculated for 5 areas
1) Left front lane
2) current lane, upfront
3) right lane, upfront
4) left lane and behind
5) right lane and behind

Safety and collision
 All states first check if the object detected within the same lane requires the
 vehicle to slow down. The follow distance is maintained and the velocity of the
 vehicle upfront is obtained to which the velocity of our vehicle  is set.

lane changes
 Only when preparing to change lanes, additional tests are done. Costs are
 compared and the vehicle will try to move or stay into ta lane that has the
 lowest cost. When doing a lane change, the vehicle wait for a gap to arise
 to indicate a safe merging.

 Spline library
 Spline library is used to calculate a smooth trajectory between the future
 points. Not doing so, will result in jerks at transition points in the
 trajectory
