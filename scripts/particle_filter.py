#!/usr/bin/env python3

import rospy
import copy

from likelihood_field import LikelihoodField

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Quaternion, Point, Pose, PoseArray, PoseStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header, String

import tf
from tf import TransformListener
from tf import TransformBroadcaster
from tf.transformations import quaternion_from_euler, euler_from_quaternion

import numpy as np
from numpy.random import random_sample, choice
import math

from random import randint, random

def get_yaw_from_pose(p):
    """ A helper function that takes in a Pose object (geometry_msgs) and returns yaw"""

    yaw = (euler_from_quaternion([
            p.orientation.x,
            p.orientation.y,
            p.orientation.z,
            p.orientation.w])
            [2])

    return yaw


def draw_random_sample(sample, weight):
    """ Draws a random sample of n elements from a given list of choices and their specified probabilities.

    Uses numpy's random choice function.
    Particles must be deep copied because they are modified in place
    when updating the motion model.
    """
    new_cloud =  choice(sample, p=weight, size=len(sample))
    new_cloud = [copy.deepcopy(p) for p in new_cloud]
    return new_cloud

def compute_prob_zero_centered_gaussian(dist, sd):
    """ Takes in distance from zero (dist) and standard deviation (sd) for gaussian
        and returns probability (likelihood) of observation 

    Taken from the starter code provided in class. """
    c = 1.0 / (sd * math.sqrt(2 * math.pi))
    prob = c * math.exp((-math.pow(dist,2))/(2 * math.pow(sd, 2)))
    return prob

class Particle:

    def __init__(self, pose, w):

        # particle pose (Pose object from geometry_msgs)
        self.pose = pose

        # particle weight
        self.w = w

    def __repr__(self):
        return f"({self.pose.position.x}, {self.pose.position.y})"



class ParticleFilter:


    def __init__(self):

        # once everything is setup initialized will be set to true
        self.initialized = False        

        # initialize this particle filter node
        rospy.init_node('turtlebot3_particle_filter')

        # set the topic names and frame names
        self.base_frame = "base_footprint"
        self.map_topic = "map"
        self.odom_frame = "odom"
        self.scan_topic = "scan"

        # initialize our map
        self.map = OccupancyGrid()

        # the number of particles used in the particle filter
        self.num_particles = 10000

        # initialize the particle cloud array
        self.particle_cloud = []

        # initialize the estimated robot pose
        self.robot_estimate = Pose()

        # set threshold values for linear and angular movement before we preform an update
        self.lin_mvmt_threshold = 0.2        
        self.ang_mvmt_threshold = (np.pi / 6)

        self.odom_pose_last_motion_update = None

        # initialize the likelihood field
        self.likelihood_field = LikelihoodField()

        # Setup publishers and subscribers

        # publish the current particle cloud
        self.particles_pub = rospy.Publisher("particle_cloud", PoseArray, queue_size=10)

        # publish the estimated robot pose
        self.robot_estimate_pub = rospy.Publisher("estimated_robot_pose", PoseStamped, queue_size=10)

        # subscribe to the map server
        rospy.Subscriber(self.map_topic, OccupancyGrid, self.get_map)

        # subscribe to the lidar scan from the robot
        rospy.Subscriber(self.scan_topic, LaserScan, self.robot_scan_received)

        # enable listening for and broadcasting corodinate transforms
        self.tf_listener = TransformListener()
        self.tf_broadcaster = TransformBroadcaster()


        # intialize the particle cloud
        self.initialize_particle_cloud()

        self.initialized = True



    def get_map(self, data):

        self.map = data
    

    def initialize_particle_cloud(self):
        """
        Generates the initial particle cloud
        with particles placed randomly in 
        open spaces in the map.
        """
        # find all of the open tiles, and then pick 10000 random ones 
        open_tiles = [] 
        for i in range(self.map.info.width):
            for j in range(self.map.info.height):
                ind = i + j*self.map.info.width
                if self.map.data[ind] == 0:
                    open_tiles.append((i, j))

        for tile in np.random.choice(range(len(open_tiles)), size=self.num_particles):
            # Select tiles by index, because numpy complains about
            # multi-dimensional data if we call `choice` directly
            # only the `open_tiles` array
            i, j = open_tiles[tile]
            pose = Pose()
            # Generate random position and yaw, and create a particle
            pose.position.x = i * self.map.info.resolution + self.map.info.origin.position.x
            pose.position.y = j * self.map.info.resolution + self.map.info.origin.position.y
            pose.position.z = 0
            q_x, q_y, q_z, q_w = quaternion_from_euler(0, 0, random() * 2 * math.pi)
            pose.orientation = Quaternion(q_x, q_y, q_z, q_w)
            p = Particle(pose, 1 / self.num_particles)
            self.particle_cloud.append(p)

        self.normalize_particles()

        self.publish_particle_cloud()


    def normalize_particles(self):
        """
        Make all the particle weights sum to 1.0
        """
        total_weight = np.sum([p.w for p in self.particle_cloud])
        for p in self.particle_cloud:
            p.w = p.w / total_weight


    def publish_particle_cloud(self):
        particle_cloud_pose_array = PoseArray()
        particle_cloud_pose_array.header = Header(stamp=rospy.Time.now(), frame_id=self.map_topic)
        particle_cloud_pose_array.poses = []

        for part in self.particle_cloud:
            particle_cloud_pose_array.poses.append(part.pose)

        self.particles_pub.publish(particle_cloud_pose_array)




    def publish_estimated_robot_pose(self):

        robot_pose_estimate_stamped = PoseStamped()
        robot_pose_estimate_stamped.pose = self.robot_estimate
        robot_pose_estimate_stamped.header = Header(stamp=rospy.Time.now(), frame_id=self.map_topic)
        self.robot_estimate_pub.publish(robot_pose_estimate_stamped)



    def resample_particles(self):
        # Resample a new particle cloud using the weights
        # of the particles
        self.particle_cloud = draw_random_sample(self.particle_cloud, [p.w for p in self.particle_cloud])


    def robot_scan_received(self, data):

        # wait until initialization is complete
        if not(self.initialized):
            return

        # we need to be able to transform the laser frame to the base frame
        if not(self.tf_listener.canTransform(self.base_frame, data.header.frame_id, data.header.stamp)):
            return

        # wait for a little bit for the transform to become available (in case the scan arrives
        # a little bit before the odom to base_footprint transform was updated) 
        self.tf_listener.waitForTransform(self.base_frame, self.odom_frame, data.header.stamp, rospy.Duration(0.5))
        if not(self.tf_listener.canTransform(self.base_frame, data.header.frame_id, data.header.stamp)):
            return

        # calculate the pose of the laser distance sensor 
        p = PoseStamped(
            header=Header(stamp=rospy.Time(0),
                          frame_id=data.header.frame_id))

        self.laser_pose = self.tf_listener.transformPose(self.base_frame, p)

        # determine where the robot thinks it is based on its odometry
        p = PoseStamped(
            header=Header(stamp=data.header.stamp,
                          frame_id=self.base_frame),
            pose=Pose())

        self.odom_pose = self.tf_listener.transformPose(self.odom_frame, p)

        # we need to be able to compare the current odom pose to the prior odom pose
        # if there isn't a prior odom pose, set the odom_pose variable to the current pose
        if not self.odom_pose_last_motion_update:
            self.odom_pose_last_motion_update = self.odom_pose
            return


        if self.particle_cloud is not None:

            # check to see if we've moved far enough to perform an update

            curr_x = self.odom_pose.pose.position.x
            old_x = self.odom_pose_last_motion_update.pose.position.x
            curr_y = self.odom_pose.pose.position.y
            old_y = self.odom_pose_last_motion_update.pose.position.y
            curr_yaw = get_yaw_from_pose(self.odom_pose.pose)
            old_yaw = get_yaw_from_pose(self.odom_pose_last_motion_update.pose)

            if (np.abs(curr_x - old_x) > self.lin_mvmt_threshold or 
                np.abs(curr_y - old_y) > self.lin_mvmt_threshold or
                np.abs(curr_yaw - old_yaw) > self.ang_mvmt_threshold):

                # This is where the main logic of the particle filter is carried out

                self.update_particles_with_motion_model()

                self.update_particle_weights_with_measurement_model(data)

                self.normalize_particles()

                self.resample_particles()

                self.update_estimated_robot_pose()

                self.publish_particle_cloud()
                self.publish_estimated_robot_pose()

                self.odom_pose_last_motion_update = self.odom_pose



    def update_estimated_robot_pose(self):
        """
        Average all of the particles positions and yaws
        to produce an estimate of where the robot is.
        """
        total_x = 0
        total_y = 0
        # Yaw is averaged by converting each yaw into
        # a point on the unit circle, and then averaging those.
        # This helps avoid the situation where some yaws are 0 degrees,
        # and some are 359 degrees, so the average of just
        # the angles would point in the opposite direction.
        total_yaw_x = 0
        total_yaw_y = 0
        for p in self.particle_cloud:
            total_x += p.pose.position.x
            total_y += p.pose.position.y
            p_yaw = get_yaw_from_pose(p.pose)
            total_yaw_x += math.cos(p_yaw)
            total_yaw_y += math.sin(p_yaw)
        average_yaw = math.atan2(total_yaw_y, total_yaw_x)
        self.robot_estimate.position.x = total_x / self.num_particles
        self.robot_estimate.position.y = total_y / self.num_particles
        q_x, q_y, q_z, q_w = quaternion_from_euler(0, 0, average_yaw)
        self.robot_estimate.orientation = Quaternion(q_x, q_y, q_z, q_w) 

    
    def update_particle_weights_with_measurement_model(self, data):
        """
        Computes new weights for each particle based on
        the robot's LIDAR data
        """
        # We experimented with different amounts of angles,
        # and 8 seemed to provided the best accuracy and consistency.
        directions_idxs = [0,45,90,135,180,225,270,315]

        # For each particle, compute its weight using 
        # a likelihood field
        for particle in self.particle_cloud:
            p_theta = euler_from_quaternion([
                particle.pose.orientation.x,
                particle.pose.orientation.y,
                particle.pose.orientation.z,
                particle.pose.orientation.w])[2]
            p_x = particle.pose.position.x
            p_y = particle.pose.position.y
            q = 1 # initial particle weight
            for direction in directions_idxs:
                sensor_distance = data.ranges[direction]
                if sensor_distance > data.range_max:                  
                    # If the sensor is too far away to see 
                    # anything, we can't get any useful data from it,
                    # so instead just weight this particle lower.
                    q = q * 0.05
                    continue
                x_z = p_x + sensor_distance * math.cos(p_theta + math.radians(direction))
                y_z = p_y + sensor_distance * math.sin(p_theta + math.radians(direction))
                nearest_obstacle = self.likelihood_field.get_closest_obstacle_distance(x_z, y_z)
                # Lower the weight based on how well the sensor measurement lines up
                # with the likelihood field. The standard deviation of 0.1 seems like 
                # a reasonable value given the variation we have observed in the robots
                # LIDAR. It has been working fairly well for us in practice.
                q = q * compute_prob_zero_centered_gaussian(nearest_obstacle, 0.1)
            # To avoid nans, and also situations where all weights are 0,
            # replace these weights with a very small weight.
            if math.isnan(q) or q == 0:
                q = 0.000001
            particle.w = q
        

    def update_particles_with_motion_model(self):
        """
        Moves the particles based on how the robot has
        moved according to its odometry data.
        """
        # We experimented with different noise constants,
        # and found that this one provided fairly robust results
        # while also keeping the particle cloud rather tight.
        noise_const = 0.05

        # Get the robots movement from its odometry
        curr_x = self.odom_pose.pose.position.x
        old_x = self.odom_pose_last_motion_update.pose.position.x
        curr_y = self.odom_pose.pose.position.y
        old_y = self.odom_pose_last_motion_update.pose.position.y
        curr_yaw = get_yaw_from_pose(self.odom_pose.pose)
        old_yaw = get_yaw_from_pose(self.odom_pose_last_motion_update.pose)

        for particle in self.particle_cloud:
            # Compute the movement, plus a bit of uniform noise
            delta_x = curr_x - old_x + random() * noise_const - noise_const/2
            delta_y = curr_y - old_y + random() * noise_const - noise_const/2
            delta_yaw = curr_yaw - old_yaw + random() * noise_const - noise_const/2

            roll, pitch, yaw = euler_from_quaternion([particle.pose.orientation.x, particle.pose.orientation.y, particle.pose.orientation.z, particle.pose.orientation.w])
            
            # How far should the particle rotate
            angle = yaw - old_yaw
            particle.pose.position.x += delta_x * math.cos(angle) - delta_y * math.sin(angle)
            particle.pose.position.y += delta_x * math.sin(angle) + delta_y * math.cos(angle)
            yaw += delta_yaw
            if yaw > 2 * math.pi:
                yaw -= 2 * math.pi
            if yaw < 0:
                yaw += 2 * math.pi
            q_x, q_y, q_z, q_w = quaternion_from_euler(roll, pitch, yaw)
            particle.pose.orientation = Quaternion(q_x, q_y, q_z, q_w)

if __name__=="__main__":
    pf = ParticleFilter()
    rospy.spin()
