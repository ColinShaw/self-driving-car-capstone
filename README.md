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



### Traffic light and motion control integration



### How we managed work in our team



### Lessons learned and areas for improvement

