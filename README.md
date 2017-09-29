# SDC Fun

This is the `SDC Fun` team repo for the capstone project of Udacity's Self-Driving Car Engineer Nanodegree. The team members are:

 * James Barfield (jamesbarfield87@gmail.com)
 * Nick Condo(nicholas.condo@gmail.com)
 * Jim Duan (jd@vehicular.ai)
 * Nimish Sanghi (nimish.sanghi@gmail.com)
 * Colin Shaw (colin.shaw@aya.yale.edu)

### Building and running the project

The original project repo can be found [here](https://github.com/udacity/CarND-Capstone),
which has a lot of information about environment, simulator, etc.

This project requires the GPU version of Tensorflow 1.2.1 along with the CUDA and cuDNN libraries, python 2.7, as well as the following libraries:

  * Flask
  * attrdict
  * eventlet
  * python-socketio
  * numpy
  * Pillow
  * scipy
  * keras
  * h5py
  * requests

These modules can be installed using: `pip install -r requirements.txt` from the root of the project.  

You will need to download our model in order to detect traffic lights. You can download the model by navigating to `self-driving-car-capstone/data_science/models` and running `python get_model.py`.
Alternatively, you can download the model manually [here](https://drive.google.com/open?id=0B1TNQWukG_RDQ2JXUGdYQTlsSzA).

Next go into the `/ros` directory and run `catkin_make` to build the ROS project. Be sure to source the project by running `. devel/setup.sh`.  At this point, if your environment is set up properly, you should be able to launch ROS with `roslaunch launch/styx.launch`.

Further information about our implementation can be found [here](docs/ProjectSummary.pdf).
