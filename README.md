# particle_filter_localization_project

Team members: Evan Casey, Alex Oro

## Initialize particle cloud

We will iterate over the occupancy grid provided,
and randomly place particles at places where the 
robot could be (i.e. probability of occupancy > 0).
These particles will have random yaw as well.

## Update position of particles

We will use the odometry data to calculate the
robot's movement, and apply that movement to each
particle. 

## Update particle weights

For each particle, we will compute hypothetical
LIDAR scan data for that particle. Then we will
use a measurement model to determine how close each
hypothetical measurement is to our actual measurement.
We will figure out more specific after working with
these measurement models in class on 4/14/2021.

## Normalize and Resample particle weights

We will normalize the particle weights by dividing
each weight by the sum of all of the weights. We will 
then use Python's random library to resample from this
set using these weights as the likelihoods for each particle.

## Update estimated robot pose

We will average the position and yaw of each particle
to compute the robot's pose.

## Noise

We will introduce noise when moving particles; we will 
add a small amount of noise to each position and yaw.


## Timeline

We plan to finish up to updating particle position 
by Wednesday evening. Then we will finish up to 
resampling based on particle weights on Friday.
Then early next week we will work on the write up,
and make any finishing touches/debugging.
