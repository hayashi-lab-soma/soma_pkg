import numpy as np
# import fastslam as fs


class Particle:
    def __init__(self, pose, weight, features=[]):
        self.pose = np.array(pose[:])
        self.weight = weight
        self.features = features[:]

        return

    def __str__(self):
        tmp = "\nParticle for online SLAM\n"
        tmp += "\nPose: " + str(self.pose)
        tmp += "\nWeight: " + str(self.weight)
        tmp += "\nFeatures: "
        for i, f in enumerate(self.features):
            tmp += "\n" + str(i) + ": " + str(f)
        tmp += "\n"

        return tmp


class OnlineSLAMSolver:
    def __init__(self, initial_x=0.0, initial_y=0.0, initial_theta=0.0, v_noise=[0.01, 0.0], omega_noise=[0.0, 0.01], yaw_noise=[0.0, 0.01], visibility=30, d_noise=[0.05, 0.0], phi_noise=[0.0005, 0.0], particles_num=100):
        # Initial pose
        self.robot_initial_pose = np.array(
            [initial_x, initial_y, initial_theta])

        # Motion
        self.motion_noise = np.array([v_noise, omega_noise, yaw_noise])

        # Observation
        self.visibility = visibility
        self.observation_noise = np.array([d_noise, phi_noise])

        # Particles
        self.particles_num = particles_num
        self.particles = []

        for i in range(particles_num):
            # Pose
            new_pose = np.array(self.robot_initial_pose[:])

            # Weight
            new_weight = 1.0/particles_num

            # Featured-map
            new_features = np.array([])

            new_particle = Particle(new_pose, new_weight, new_features)
            self.particles.append(new_particle)

        return

    def __str__(self):
        tmp = "\nOnlineSLAMSolver from FASTSLAM\n"
        tmp += "\nRobot initial pose: " + str(self.robot_initial_pose)
        tmp += "\nMotion noise:\n" + str(self.motion_noise)
        tmp += "\nVisibility: " + str(self.visibility)
        tmp += "\nObservation noise:\n" + str(self.observation_noise)
        tmp += "\nNumber of particles: " + str(self.particles_num) + "\n"

        return tmp


# Tests

P = Particle(np.array([2.4, -5.6, 0.1254]), 0.0015,
             [np.array([78.9, -14.26]), np.array([4.56, 12.9])])
print(P)

S = OnlineSLAMSolver()
print(S)
