# particle_filter_localization_project

Team members: Evan Casey, Alex Oro

![Recording](./recording.gif)

## Objectives

To design and implement a robot localization algorithm based off of a particle cloud.

**High Level Description**

We were able to solve the problem of localization by using a likelihood filter similar to the one we implemented in class. We improved on the filter we wrote in class by increasing the number of angles that we check at each particle. We were able to approximate the robot’s movement by using the provided odometry data and added some noise to allow for imperfections in the simulation and sensor measurements. We then estimated the robots position by averaging the position and orientation of all the particles in the cloud.

## Main Steps

**Initialization of Particle Cloud**

Occurs in the function *initialize_particle_cloud()*.

We achieve this by first creating a list of tiles that are empty and thus are potential possible locations for the robot to be. Then we use numpy’s *random.choice()* function to choose a random selection of 10000 positions from this list. Lastly we create a random angle and use it to create a random orientation for each particle.

**Movement Model**

Occurs in the function *update_particles_with_motion_model()*

First we get the current and old position and orientation of the robot. Then for each particle in the cloud, we calculate the distance the robot traveled based on the odometry and use *random()* to apply a flat noise constant to these distances. We then do the same for the orientation of each particle based on the robot’s current and last orientation, with the same noise ranges.


**Measurement Model**

Occurs in the function *update_particle_weights_with_measurement_model()*

First we calculate the yaw and (x,y) position of each of the particles in the cloud. Then for every 45 degrees we get the sensor measurement at that angle. Then we use *compute_prob_zero_centered_gaussian()* on the nearest found obstacle to get a probability for the weight. We found the nearest obstacle in  *get_closest_obstacle_distance()* with the calculated positions at the end of the sensor measurement, *x_z* and *y_z*.

**Resampling**

This occurs in *normalize_particles()*, and *resample_particles()*.

We use normalize the particle weights by calling *normalize_particles()* to ensure that the sum of particle weights is 1 so that when we resample we can call *draw_random_sample()* in *resample_particles()* to get a new selection of particles for the cloud based on their weights. *draw_random_sample()* uses the weights of each particle to apply numpy’s *choice()* function so that we can create a new cloud with overlapping particles.

**Incorporating Noise**

Occurs in the function *update_particles_with_motion_model()*

We add a small amount of uniform noise to the movements obtained from the robot’s odometry data. We add this noise to both the change in x and y coordinates, and the change in yaw.

**Updating estimated robot pose**

Occurs in the function *update_estimated_robot_pose()*

We average all of the particle positions to produce the estimated robot position. For yaw, we first convert each particle’s yaw into a point on the unit circle, and then average those to produce the robot’s estimated yaw. This helps avoid issues where two angles can be very close together, but numerically are far apart (i.e. 0 and 359 degrees), so the average points the opposite way.

**Optimization of Parameters**

| Parameter              | Code Location                                    | Value                         | Description                                                                      |
|------------------------|--------------------------------------------------|-------------------------------|----------------------------------------------------------------------------------|
| *num_particles*          | *__init__()*                                       | 10000                         | How many particles are in the cloud                                              |
| *directions_idxs*        | *update_particle_weights_with_measurement_model()* | [0,45,90,135,180,225,270,315] | Which angles should the robot read sensor data from                              |
| *inf_measurement_weight* | *update_particle_weights_with_measurement_model()* | 0.05                          | How much to scale the weight of a particle when a sensor measurement is infinite |
| *measurement_sd*         | *update_particle_weights_with_measurement_model()* | 0.1                           | Standard deviation in sensor measurements                                        |
| *small_weight*           | *update_particle_weights_with_measurement_model()* | 0.000001                      | If the particle weight is 0, replace it with this                                |
| *noise_const*            | *update_particles_with_motion_model()*             | 0.05                          | How much noise to introduce in motion                                            |


Above is a table of all of our constants. We modified these slightly as we introduced each one to see what values were reasonable. With our current weighting, we have a fairly robust and accurate particle filter.

## Challenges

We encountered a few issues with floating point arithmetic, which would cause situations where all of our particle weights became 0, and then NaN. We fixed this by checking for weights of 0, and replacing them with a very small number. We also had a few issues handling yaws. One of them was where the particle yaw would not turn the same amount as the robot turned; we resolved this by working out some trigonometry on pen and paper, and realizing where our formula was wrong. We also had issues with averaging yaw, where occasionally the estimated robot pose would point the opposite direction as all of the particles. We resolved this problem by first converting yaws to points on the unit circle, and then averaging those to produce the estimated robot pose.

## Future Work

If we had more time, we would spend more time fine tuning the various parameters in our model. While our current one is fairly robust, it could probably be tighter and follow the robot more closely. Occasionally, during testing, the particle cloud would converge to a point where the robot was not. Given more time, we would investigate methods for allowing the particle cloud to correct itself in this situation, such as occasionally adding random particles on the map for robustness. And we could add more error handling for edge cases involving floating point arithmetic.

## Takeaways

* Floating point arithmetic is tricky. When dealing with floating points, it is best to include sanity checks in case any values become 0, NaN, or infinity when not expected. Catching these errors early can save a lot of debugging time.
* Having no noise in the particle movements leads to an inaccurate estimate of where the robot actually is since the small differences between the robot’s actual movement and what the simulation does compound over time. Adding in noise and averaging it out in the end means that these errors don’t get larger than the noise variance at any given time.

---

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
