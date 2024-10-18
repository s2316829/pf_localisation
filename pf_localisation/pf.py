from geometry_msgs.msg import Pose, PoseArray, Quaternion, Point
from . pf_base import PFLocaliserBase
from . util import rotateQuaternion, getHeading
import random
import numpy as np
from .sensor_model import SensorModel
import rospy


class PFLocaliser(PFLocaliserBase):
       
    def __init__(self):
        # ----- Call the superclass constructor
        super(PFLocaliser, self).__init__()
        
        # ----- Set motion model parameters
        self.ODOM_ROTATION_NOISE = 0.05
        self.ODOM_TRANSLATION_NOISE = 0.02
        self.ODOM_DRIFT_NOISE = 0.01
        # ----- Sensor model parameters
        self.NUMBER_PREDICTED_READINGS = 20     # Number of readings to predict
        

    
    def initialise_particle_cloud(self, initialpose):
        """
        Called when setting the initial position of the particle cloud.
        """
        particle_cloud = PoseArray()
        particle_cloud.header = initialpose.header

        for _ in range(self.num_particles):
            # Create each particle with some noise around the initial pose
            particle = Pose()

            # Random Gaussian noise around the initial pose
            particle.position.x = initialpose.pose.pose.position.x + random.gauss(0, 0.5)
            particle.position.y = initialpose.pose.pose.position.y + random.gauss(0, 0.5)
            particle.position.z = 0.0  # assuming the robot is on a flat surface

            # Random noise for orientation
            yaw_noise = random.gauss(0, 0.1)
            particle.orientation = rotateQuaternion(initialpose.pose.pose.orientation, yaw_noise)

            particle_cloud.poses.append(particle)

        self.particlecloud = particle_cloud
        return particle_cloud


    def update_particle_cloud(self, scan):
        """
        Called whenever a new LaserScan message is received.
        """
        weights = []

        # Calculate weight for each particle based on the laser scan data
        for particle in self.particlecloud.poses:
            weight = self.sensor_model.get_weight(scan, particle)
            weights.append(weight)

        # Normalize weights to create a probability distribution
        total_weight = sum(weights)
        if total_weight == 0:
            rospy.logwarn("All particles have zero weight, reinitializing.")
            weights = [1.0 / self.num_particles] * self.num_particles
        else:
            weights = [weight / total_weight for weight in weights]

        # Resample the particles based on their weights
        new_particle_cloud = PoseArray()
        new_particle_cloud.header = self.particlecloud.header

        indices = np.random.choice(range(self.num_particles), size=self.num_particles, p=weights)
        for index in indices:
            selected_particle = self.particlecloud.poses[index]

            # Add some noise to keep particle diversity
            new_particle = Pose()
            new_particle.position.x = selected_particle.position.x + random.gauss(0, 0.1)
            new_particle.position.y = selected_particle.position.y + random.gauss(0, 0.1)
            new_particle.orientation = rotateQuaternion(selected_particle.orientation, random.gauss(0, 0.05))

            new_particle_cloud.poses.append(new_particle)

        self.particlecloud = new_particle_cloud


    def estimate_pose(self):
        """
        Estimate the robot's pose from the particle cloud.
        """
        # Calculate the weighted average of all particles to determine the robot's pose
        average_pose = Pose()
        x_sum = 0.0
        y_sum = 0.0
        sin_sum = 0.0
        cos_sum = 0.0

        for particle in self.particlecloud.poses:
            x_sum += particle.position.x
            y_sum += particle.position.y

            # Convert quaternion to yaw to accumulate
            yaw = getHeading(particle.orientation)
            sin_sum += np.sin(yaw)
            cos_sum += np.cos(yaw)

        average_pose.position.x = x_sum / self.num_particles
        average_pose.position.y = y_sum / self.num_particles

        # Convert back to quaternion
        average_yaw = np.arctan2(sin_sum, cos_sum)
        average_pose.orientation = rotateQuaternion(self.particlecloud.poses[0].orientation, average_yaw)

        return average_pose
