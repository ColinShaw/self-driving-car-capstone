# Udacity Capstone Project 

This is our `SDC Fun` team repo for the capstone project in Udacity's Self-Driving Car Nanodegree 
program.  Our team members are:

 * James Barfield (jamesbarfield87@gmail.com)
 * Nick Condo(nicholas.condo@gmail.com)
 * Jim Duan (jd@vehicular.ai)
 * Nimish Sanghi (nimish.sanghi@gmail.com)
 * Colin Shaw (colin.shaw@aya.yale.edu)



### Original sources

The original project repo can be found [here](https://github.com/udacity/CarND-Capstone), 
which has a lot of information about environment, simulator, etc.



### Building and running the project

This project depends on having CUDA and cuDNN installed in order to function
properly.  Reason is the inference speed for traffic light detection.  You will
notice a `requirements.txt` file in the root of the project.  If your machine does
not satisfy the above requirement, you will need to change the `tensorflow-gpu`
requirement to `tensorflow` to support CPU alone.  All other (python) dependencies
can be satisfied from the root of the project by `pip install -r requirements.txt`.  

Next go into the `/ros` directory and run `catkin_make`.  Be sure to source the 
project by running `. devel/setup.sh`.  At this point, if your environment is set 
up properly, you should be able to launch ROS with `roslaunch launch/styx.launch`.



### Traffic light detection



### Motion control

Motion control depends on knowledge of the waypoints.  In this case, the waypoints
are sufficiently close together to not require interpolation.  We first changed
`waypoint_loader.py` to publish a single, latched read of the waypoints, as 
it had initially been set to publish remarkably frequently and wasted a lot of
valuable resources.  This was done with latching a single `publish()` because 
the use of the topic is essentially to avail a static data set to various objects
that use the data as a property.  Doing this simply improved performance.

The `waypoint_updater.py` code is critical to the project as it performs two main 
tasks.  First, it takes the current pose and other runtime properties, as well
as the static waypoints, and produces a `Lane` topic that consists of a subset of 
the waypoints starting with the first waypoint in front of the car, with a twist 
linear velocity set in the axis of travel.  Second, the traffic light control 
is performed, which is the topic of the next section.

We first identify which waypoint index is nearest in front of the car.  Next, 
depending on the control from the traffic light, we generate a velocity profile 
for waypoints in front of the car.  These are combined in a `Lane` topic 
and published.  We added a new topic that is intended to be invoked from the 
console using `rostopic pub ...` so that the speed setpoint can be adjusted at whim.  

The waypoint follower implements what is known as the `pure pursuit` algorithm
(see [here](docs/Coulter.pdf) for more on that), which returns a `TwistStamped`
topic that is the computed best move for the car based on the algorithm.  We noted
that the implementation of the algorithm indexes the waypoints proportional to 
the speed of travel, so increased the `LOOKAHEAD_WPS` constant in `waypoint_updater.py`
to facilitate higher speed travel.  While we are aware the speeds that the
real car will travel are not as high, we wanted to be able to tune our simulated
car for reasonable travel at highway speeds.

`dbw_node.py` subscribes to the `TwistStamped` topic produced by the waypoint 
follower.  Here we seek to control the throttle, brake and steering of the 
car using the desired linear and twist velocities and the current velocity.  The 
main loop in `dbw_node.py` simply creates a convenient data structure that is
passed to an instance of our `Controller` class to achieve the desired control.

With respect to the throttle and brake, we implemented a single PID 
controller that has a split output for the throttle and brake.  This also allows
slightly higher impetus to the brake, as we want the car to be able to stop
more quickly in some cases than we generally would want for acceleration.  It 
also makes it simple to accommodate the brake deadband.  The PID controller was
tuned manually (see [here](docs/BasilioMatos.pdf) and 
[here](docs/ZieglerNichols.pdf) for more on tuning), including a small integral 
component that helps with acceleration.  It was noticed that the integral history 
is harmful to the stopping dynamics, so when the stopping action is started, the 
integral history is reset (also done if the `dbw_enable` topic is disabled).  

The steering control is more complex.  We compute a desired steering and current 
steering estimate using an instance of the `YawController` class with our 
desired and current steering information from the waypoint follower and current 
pose.  We pre-filter the steering error using a minimal low pass filter, and 
then submit this to a PID controller.  This controller was also manually 
tuned, and includes a more significant integral component that helps it track the
higher curvature sections of road better.  The PID controller is followed by 
a second low pass filter having greater effect, which smooths the resulting 
steering command sent to the car.  The specific constants for the two low pass 
filters were manually selected to have the best smoothing effect while not 
interfering with the ability to steer on the sharpest turns.  



### Traffic light and motion control integration



### How we managed work in our team

One important aspect of working on a team to complete this project was
defining a good schedule with milestones for completion.  We wanted to
have an MVP ready to turn in by September 18, two full weeks prior to 
the project deadline, so that if there were any problems, unmet
requirements or new ideas there would be plenty of time to accommodate
so that we would be able to have our code run on the actual car.  When
we intitially formed the team we decided this accelerated schedule was 
desirable since we all wanted to achieve this shared goal.

In terms of work division, this project has two main components, training 
a model to identify the waypoint associated with the nearest red light 
in the direction of travel of the car, and waypoint management and motion 
control.  These tasks can be divided into a sequence of two asynchronous 
tasks followed by integration and testing. 

The members of our team selected what aspect to work on initially based
on personal preference, knowing that as the project evolved we would all
be involved in a wider view of it.  James and Nick predominantly worked 
on the model for the traffic light detector while Colin and Nimish 
predominantly worked on the motion control.  By the weekend of September 9
we were able to start working on integration to meet our MVP milestone.

Our team predominately used Slack to facilitate communications, augmented
by e-mail correspondence and a few video conference calls used to meet each
other and check in on progress during the integration phase.  We opted to 
use a single GitHub repo with privileges granted all team members.  



### Lessons learned and areas for improvement

