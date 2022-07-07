import numpy as np
import pybullet as p
import time
import os
import warnings
import matplotlib.pyplot as plt

class RobotSimulator:
    def __init__(self, dt=0.04, display=True, stars=None, shootingstar=True, seed=None, scope_noise=0.1):
        # Create random number generator
        self.rng = np.random.default_rng(seed)

        # Choose the time step
        self.dt = dt

        # Connect to and configure pybullet
        self.display = display
        if self.display:
            p.connect(p.GUI, options="--background_color_red=0 --background_color_blue=0 --background_color_green=0")
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
            # p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING, 1)
            p.resetDebugVisualizerCamera(6, -90, -45, (0., 0., 0.))
        else:
            p.connect(p.DIRECT)
        p.setPhysicsEngineParameter(fixedTimeStep=self.dt,
                                    numSubSteps=4,
                                    restitutionVelocityThreshold=0.05)

        # Load robot
        self.robot_id = p.loadURDF(os.path.join('.', 'urdf', 'spacecraft.urdf'),
                                   basePosition=np.array([0., 0., 0.]),
                                   baseOrientation=p.getQuaternionFromEuler([0., 0., 0.]),
                                   useFixedBase=0,
                                   flags=(p.URDF_USE_IMPLICIT_CYLINDER  |
                                          p.URDF_USE_INERTIA_FROM_FILE  ))

        # Load shooting star
        self.shootingstar = shootingstar
        if self.shootingstar:
            self.shot_id = p.loadURDF(os.path.join('.', 'urdf', 'shootingstar.urdf'),
                                       basePosition=np.array([0., 0., 10.]),
                                       baseOrientation=p.getQuaternionFromEuler([0., 0., 0.]),
                                       useFixedBase=0,
                                       flags=(p.URDF_USE_IMPLICIT_CYLINDER  |
                                              p.URDF_USE_INERTIA_FROM_FILE  ))
            p.changeDynamics(self.shot_id, -1, linearDamping=0., angularDamping=0.)

        # Eliminate linear and angular damping (a poor model of drag)
        p.changeDynamics(self.robot_id, -1, linearDamping=0., angularDamping=0.)
        for joint_id in range(p.getNumJoints(self.robot_id)):
            p.changeDynamics(self.robot_id, joint_id, linearDamping=0., angularDamping=0.)

        # Eliminate joint damping
        for joint_index in range(p.getNumJoints(self.robot_id)):
            p.changeDynamics(self.robot_id, joint_index, jointDamping=0.)

        # Create a dictionary that maps joint names to joint indices
        self.joint_map = {}
        for joint_index in range(p.getNumJoints(self.robot_id)):
            joint_name = p.getJointInfo(self.robot_id, joint_index)[1].decode('UTF-8')
            self.joint_map[joint_name] = joint_index

        # Create a 1D numpy array with the index (according to bullet) of each joint we care about
        self.joint_names = [
            'bus_to_wheel_1',
            'bus_to_wheel_2',
            'bus_to_wheel_3',
            'bus_to_wheel_4',
        ]
        self.num_joints = len(self.joint_names)
        self.joint_ids = np.array([self.joint_map[joint_name] for joint_name in self.joint_names])

        # Disable velocity control on joints so we can use torque control
        p.setJointMotorControlArray(self.robot_id, self.joint_ids,
                                    p.VELOCITY_CONTROL, forces=np.zeros(self.num_joints))

        # Specify maximum applied torque
        self.tau_max = 5.

        # Specify maximum wheel speed for non-zero torque
        # (20 rad/s is about 200 rpm)
        self.v_max = 20.

        # Place stars
        self.scope_radius = 0.8 / 2.1
        self.scope_angle = np.arctan(self.scope_radius)
        self.scope_noise = scope_noise
        self.star_depth = 5.
        if stars is None:
            stars = np.array([[0., 0.], [0.15, 0.], [0., 0.15]])
        else:
            stars = np.array(stars)
            if (len(stars.shape) != 2) or (stars.shape[1] != 2):
                raise Exception('"stars" must be a numpy array of size n x 2')
        self.stars = []
        for i in range(stars.shape[0]):
            self.stars.append({'alpha': stars[i, 0], 'delta': stars[i, 1],})
        for star in self.stars:
            star['pos'] = np.array([[np.cos(star['alpha']) * np.cos(star['delta'])],
                                    [np.sin(star['alpha']) * np.cos(star['delta'])],
                                    [np.sin(star['delta'])]]) * self.star_depth
            p.loadURDF(os.path.join('.', 'urdf', 'sphere.urdf'),
                                    basePosition=star['pos'].flatten(),
                                    useFixedBase=1)

        if self.shootingstar:
            for object_id in [self.robot_id, self.shot_id]:
                for joint_id in range(-1, p.getNumJoints(object_id)):
                    p.changeDynamics(object_id, joint_id,
                        lateralFriction=1.0,
                        spinningFriction=0.0,
                        rollingFriction=0.0,
                        restitution=0.5,
                        contactDamping=-1,
                        contactStiffness=-1)

    def disconnect(self):
        p.disconnect()

    def get_sensor_measurements(self):
        """
        returns two numpy arrays:
        - shape (n, 2) with image coordinates of each star (or nan if out of scope)
        - shape (4, ) with angular velocity of each reaction wheel
        """
        # angular velocity of each reaction wheel
        joint_states = p.getJointStates(self.robot_id, self.joint_ids)
        v = np.zeros(self.num_joints)
        for i in range(self.num_joints):
            v[i] = joint_states[i][1]

        # position of each star in the image frame
        pos, ori = p.getBasePositionAndOrientation(self.robot_id)
        o_body_in_world = np.reshape(np.array(pos), (3, 1))
        R_body_in_world = np.reshape(np.array(p.getMatrixFromQuaternion(ori)), (3, 3))
        pos_in_image = []
        for star in self.stars:
            pos_in_body = (R_body_in_world.T @ (-o_body_in_world + star['pos'])).flatten()
            star['y'] = (pos_in_body[1] / pos_in_body[0]) / self.scope_radius
            star['z'] = (pos_in_body[2] / pos_in_body[0]) / self.scope_radius
            if (star['y']**2 + star['z']**2) <= 1.:
                pos_in_image.append([star['y'], star['z']])
            else:
                pos_in_image.append([np.nan, np.nan])

        pos_in_image = np.array(pos_in_image)
        pos_in_image += self.scope_noise * self.rng.standard_normal(pos_in_image.shape)

        return pos_in_image.flatten(), np.array(v)

    def get_rpy_and_angvel(self):
        """
        returns roll, pitch, yaw angles
        """
        pos, ori = p.getBasePositionAndOrientation(self.robot_id)
        rpy = p.getEulerFromQuaternion(ori)
        vel = p.getBaseVelocity(self.robot_id)
        return rpy, vel[1]

    def set_actuator_commands(self, tau_desired):
        tau_desired = np.array(tau_desired)
        if np.isnan(tau_desired).any():
            warnings.warn(f'invalid actuator commands: tau = {tau_desired} (setting torques to zero)', stacklevel=2)
            tau_desired = np.zeros(self.num_joints)
        tau = np.clip(tau_desired, -self.tau_max, self.tau_max)

        # Zero torque if wheel is spinning too fast
        joint_states = p.getJointStates(self.robot_id, self.joint_ids)
        for i in range(self.num_joints):
            v = joint_states[i][1]
            if (v > self.v_max) and (tau[i] > 0):
                tau[i] = 0.
            elif (v < -self.v_max) and (tau[i] < 0):
                tau[i] = 0.

        self.set_joint_torque(tau)
        return tau

    def place_shootingstar(self):
        pos = self.rng.uniform([-2., -2., 5.], [2., 2., 15.])
        v = -5.
        if self.rng.choice([True, False]):
            pos[2] = -pos[2]
            v = -v
        p.resetBasePositionAndOrientation(self.shot_id,
                                          pos,
                                          p.getQuaternionFromEuler([0., 0., 0.]))
        p.resetBaseVelocity(self.shot_id,
                            linearVelocity=[0., 0., v],
                            angularVelocity=[0., 0., 0.])

    def reset(self, rpy=None, angvel=None, scope_noise=None):
        # scope noise (if specified)
        if scope_noise is not None:
            self.scope_noise = scope_noise

        # reaction wheels
        q = np.zeros(self.num_joints)
        v = np.zeros(self.num_joints)
        for i, joint_id in enumerate(self.joint_ids):
            p.resetJointState(self.robot_id, joint_id, q[i], v[i])

        # base position, orientation, and velocity
        pos = np.array([0., 0., 0.])
        if rpy is None:
            rpy = 0.1 * self.rng.standard_normal(3)
        ori = p.getQuaternionFromEuler(rpy)
        p.resetBasePositionAndOrientation(self.robot_id, pos, ori)
        if angvel is None:
            angvel = 0.1 * self.rng.standard_normal(3)
        p.resetBaseVelocity(self.robot_id,
                            linearVelocity=[0., 0., 0.],
                            angularVelocity=angvel)

        # shooting star position, orientation, and velocity
        if self.shootingstar:
            self.place_shootingstar()


    def set_joint_torque(self, tau):
        """
        sets joint torques to values specified by the 1D numpy array tau
        """
        assert(tau.shape[0] == self.num_joints)
        zero_gains = tau.shape[0] * (0.,)
        p.setJointMotorControlArray(self.robot_id, self.joint_ids,
                                    p.TORQUE_CONTROL, forces=tau,
                                    positionGains=zero_gains, velocityGains=zero_gains)

    def step(self, t=None):
        """
        does one step in the simulation
        """

        # try to stay real-time
        if self.display:
            if t is None:
                time.sleep(self.dt)
            else:
                time_to_wait = t - time.time()
                while time_to_wait > 0:
                    time.sleep(0.9 * time_to_wait)
                    time_to_wait = t - time.time()

        # apply external force to keep spacecraft position fixed
        pos, ori = p.getBasePositionAndOrientation(self.robot_id)
        vel = p.getBaseVelocity(self.robot_id)
        f = - 150. * np.array(pos) - 50. * np.array(vel[0])
        p.applyExternalForce(self.robot_id, -1, f, pos, p.WORLD_FRAME)

        # reset shooting star
        if self.shootingstar:
            pos, ori = p.getBasePositionAndOrientation(self.shot_id)
            if np.linalg.norm(np.array(pos)) > 15:
                self.place_shootingstar()

        # take a simulation step
        p.stepSimulation()

    def snapshot(self):
        # Note: you *must* specify a projectionMatrix when calling getCameraImage,
        # or you will get whatever view is currently shown in the GUI.

        # scope view
        pos, ori = p.getBasePositionAndOrientation(self.robot_id)
        o_body_in_world = np.reshape(np.array(pos), (3, 1))
        R_body_in_world = np.reshape(np.array(p.getMatrixFromQuaternion(ori)), (3, 3))
        p_eye = o_body_in_world
        p_target = (o_body_in_world + R_body_in_world @ np.array([[10.0], [0.], [0.]])).flatten()
        v_up = (R_body_in_world[:, 2]).flatten()
        view_matrix = p.computeViewMatrix(p_eye, p_target, v_up)
        projection_matrix = p.computeProjectionMatrixFOV(fov=45.0, aspect=1.0, nearVal=0.1, farVal=10.0)
        im = p.getCameraImage(128, 128, viewMatrix=view_matrix, projectionMatrix=projection_matrix, renderer=p.ER_BULLET_HARDWARE_OPENGL, shadow=0)
        rgba_scope = im[2]

        # hack to get black background color
        depth_scope = im[3]
        for i in range(3):
            rgba_scope[:, :, i] = np.where(depth_scope >= 0.99, 0., rgba_scope[:, :, i])

        # spacecraft view
        p_eye = 1.1 * np.array([-3., -4., 4.])
        p_target = np.array([0., 0., 0.])
        v_up = np.array([0., 0., 1.])
        view_matrix = p.computeViewMatrix(p_eye, p_target, v_up)
        projection_matrix = p.computeProjectionMatrixFOV(fov=60, aspect=1.0, nearVal=1.0, farVal=20.0)
        im = p.getCameraImage(480, 480, viewMatrix=view_matrix, projectionMatrix=projection_matrix, renderer=p.ER_BULLET_HARDWARE_OPENGL, shadow=1)
        rgba_world = im[2]

        # add crosshairs to scope view
        rgba_scope[64, :, 0] = 255
        rgba_scope[64, :, 1] = 255
        rgba_scope[64, :, 2] = 255
        rgba_scope[:, 64, 0] = 255
        rgba_scope[:, 64, 1] = 255
        rgba_scope[:, 64, 2] = 255

        # hack to get black background color
        depth_world = im[3]
        for i in range(3):
            rgba_world[:, :, i] = np.where(depth_world >= 0.99, 0., rgba_world[:, :, i])

        # put scope view inside spacecraft view (picture-in-picture)
        rgba_world[10:138, 10:138, :] = rgba_scope

        return rgba_world
