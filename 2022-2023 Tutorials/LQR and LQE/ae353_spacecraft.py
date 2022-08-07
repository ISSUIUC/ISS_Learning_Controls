import numpy as np
import pybullet
import time
import os
import json
import importlib

class Simulator:
    def __init__(
            self,
            dt=0.04,
            display=True,
            stars=None,
            shootingstar=True,
            seed=None,
            scope_noise=0.1,
            width=640,
            height=480,
        ):

        # Random number generator
        self.rng = np.random.default_rng(seed)

        # Size of display
        self.width = width
        self.height = height

        # Time step
        self.dt = dt

        # Other parameters
        # - Maximum applied torque
        self.tau_max = 1.
        # - Maximum wheel speed (50 rad/s is about 500 rpm)
        self.v_max = 50.

        # Connect to and configure pybullet
        self.display = display
        if self.display:
            options = '--background_color_red=0   ' \
                    + '--background_color_blue=0  ' \
                    + '--background_color_green=0 ' \
                    + f'--width={width} --height={height}'
            pybullet.connect(pybullet.GUI, options=options)
            pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_GUI, 0)
            pybullet.resetDebugVisualizerCamera(6, -30, -40, (0., 0., 0.))
        else:
            pybullet.connect(pybullet.DIRECT)
        pybullet.setPhysicsEngineParameter(
            fixedTimeStep=self.dt,
            numSubSteps=4,
            restitutionVelocityThreshold=0.05,
            enableFileCaching=0,
        )

        # Load robot
        self.robot_id = pybullet.loadURDF(
            os.path.join('.', 'urdf', 'spacecraft.urdf'),
            basePosition=np.array([0., 0., 0.]),
            baseOrientation=pybullet.getQuaternionFromEuler([0., 0., 0.]),
            useFixedBase=0,
            flags=(pybullet.URDF_USE_IMPLICIT_CYLINDER  |
                   pybullet.URDF_USE_INERTIA_FROM_FILE  )
        )

        # Load shooting star
        self.shootingstar = shootingstar
        if self.shootingstar:
            self.shot_id = pybullet.loadURDF(
                os.path.join('.', 'urdf', 'shootingstar.urdf'),
                basePosition=np.array([0., 0., 10.]),
                baseOrientation=pybullet.getQuaternionFromEuler([0., 0., 0.]),
                useFixedBase=0,
                flags=(pybullet.URDF_USE_IMPLICIT_CYLINDER  |
                       pybullet.URDF_USE_INERTIA_FROM_FILE  )
            )
            pybullet.changeDynamics(self.shot_id, -1, linearDamping=0., angularDamping=0.)
        
        # Create a dictionary that maps joint names to joint indices and
        # link names to link indices
        self.joint_map = {}
        self.link_map = {}
        for joint_id in range(pybullet.getNumJoints(self.robot_id)):
            joint_name = pybullet.getJointInfo(self.robot_id, joint_id)[1].decode('UTF-8')
            link_name = pybullet.getJointInfo(self.robot_id, joint_id)[12].decode('UTF-8')
            self.joint_map[joint_name] = joint_id
            self.link_map[link_name] = joint_id

        # Create an array with the index of each joint we care about
        self.joint_names = [
            'bus_to_wheel_1',
            'bus_to_wheel_2',
            'bus_to_wheel_3',
            'bus_to_wheel_4',
        ]
        self.num_joints = len(self.joint_names)
        self.joint_ids = np.array([self.joint_map[joint_name] for joint_name in self.joint_names])

        # Set damping of all joints to zero
        for id in self.joint_ids:
            pybullet.changeDynamics(self.robot_id, id, jointDamping=0.)
        
        # Set other contact and damping parameters
        for object_id in [self.robot_id, self.shot_id]:
            for joint_id in range(-1, pybullet.getNumJoints(object_id)):
                pybullet.changeDynamics(
                    object_id,
                    joint_id,
                    lateralFriction=1.0,
                    spinningFriction=0.0,
                    rollingFriction=0.0,
                    restitution=0.5,
                    contactDamping=-1,
                    contactStiffness=-1,
                    linearDamping=0.,
                    angularDamping=0.,
                )
        
        # Disable velocity control on each robot joint so we can use torque control
        pybullet.setJointMotorControlArray(
            self.robot_id,
            self.joint_ids,
            pybullet.VELOCITY_CONTROL,
            forces=np.zeros(self.num_joints),
        )

        # Place stars
        self.scope_radius = 0.8 / 2.1
        self.scope_angle = np.arctan(self.scope_radius)
        self.scope_noise = scope_noise
        self.star_depth = 5.
        if stars is None:
            stars = np.array([
                [-0.10, -0.15],
                [ 0.00, -0.15],
                [ 0.10, -0.15],
                [ 0.00,  0.00],
                [-0.10,  0.15],
                [ 0.00,  0.15],
                [ 0.10,  0.15],
            ])
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
            pybullet.loadURDF(os.path.join('.', 'urdf', 'sphere.urdf'),
                                    basePosition=star['pos'].flatten(),
                                    useFixedBase=1)
    
    def get_sensor_measurements(self):
        """
        returns a 1d numpy array of length 2 * num_stars with image coordinates of star i
        (in 0, 1, ...), or nan if out of scope, at indices (2 * i) and (2 * i) + 1
        """
        
        # position of each star in the image frame
        pos, ori = pybullet.getBasePositionAndOrientation(self.robot_id)
        o_body_in_world = np.reshape(np.array(pos), (3, 1))
        R_body_in_world = np.reshape(np.array(pybullet.getMatrixFromQuaternion(ori)), (3, 3))
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

        return pos_in_image.flatten()

    def get_state(self):
        # orientation and angular velocity of spacecraft
        pos, ori = pybullet.getBasePositionAndOrientation(self.robot_id)
        rpy = pybullet.getEulerFromQuaternion(ori)
        vel = pybullet.getBaseVelocity(self.robot_id)

        # angular velocity of each reaction wheel
        joint_states = pybullet.getJointStates(self.robot_id, self.joint_ids)
        v = np.zeros(self.num_joints)
        for i in range(self.num_joints):
            v[i] = joint_states[i][1]

        return rpy, vel[1], v

    def set_actuator_commands(
            self,
            front_torque_command,
            back_torque_command,
            left_torque_command,
            right_torque_command,
        ):
        if not np.isscalar(front_torque_command):
            raise Exception('front_torque must be a scalar')
        if not np.isscalar(back_torque_command):
            raise Exception('back_torque must be a scalar')
        if not np.isscalar(left_torque_command):
            raise Exception('left_torque must be a scalar')
        if not np.isscalar(right_torque_command):
            raise Exception('right_torque must be a scalar')
        
        front_torque = np.clip(front_torque_command, -self.tau_max, self.tau_max)
        back_torque = np.clip(back_torque_command, -self.tau_max, self.tau_max)
        left_torque = np.clip(left_torque_command, -self.tau_max, self.tau_max)
        right_torque = np.clip(right_torque_command, -self.tau_max, self.tau_max)
        self.set_joint_torque(
            np.array([
                front_torque,
                back_torque,
                left_torque,
                right_torque,
            ])
        )
        return front_torque, back_torque, left_torque, right_torque
    
    def set_joint_torque(self, tau):
        if tau.shape[0] != self.num_joints:
            raise Exception('tau must be same length as number of joints')
        zero_gains = tau.shape[0] * (0.,)
        pybullet.setJointMotorControlArray(
            self.robot_id,
            self.joint_ids,
            pybullet.TORQUE_CONTROL,
            forces=tau,
            positionGains=zero_gains,
            velocityGains=zero_gains,
        )

    def place_shootingstar(self):
        pos = self.rng.uniform([-2., -2., 5.], [2., 2., 15.])
        v = -5.
        if self.rng.choice([True, False]):
            pos[2] = -pos[2]
            v = -v
        pybullet.resetBasePositionAndOrientation(self.shot_id,
                                          pos,
                                          pybullet.getQuaternionFromEuler([0., 0., 0.]))
        pybullet.resetBaseVelocity(self.shot_id,
                            linearVelocity=[0., 0., v],
                            angularVelocity=[0., 0., 0.])

    def reset(
            self,
            orientation=None,
            angular_velocity=None,
            scope_noise=None
        ):
        # Scope noise (if specified)
        if scope_noise is not None:
            self.scope_noise = scope_noise

        # Reaction wheels
        q = np.zeros(self.num_joints)
        v = np.zeros(self.num_joints)
        for i, joint_id in enumerate(self.joint_ids):
            pybullet.resetJointState(self.robot_id, joint_id, q[i], v[i])

        # Base position, orientation, and velocity
        pos = np.array([0., 0., 0.])
        if orientation is None:
            while True:
                rpy = 0.1 * self.rng.standard_normal(3)
                ori = pybullet.getQuaternionFromEuler(rpy)
                pybullet.resetBasePositionAndOrientation(self.robot_id, pos, ori)
                star_meas = self.get_sensor_measurements()
                if not np.isnan(star_meas).any():
                    break
        else:
            rpy = np.array([
                orientation['roll'],
                orientation['pitch'],
                orientation['yaw'],
            ])
            ori = pybullet.getQuaternionFromEuler(rpy)
            pybullet.resetBasePositionAndOrientation(self.robot_id, pos, ori)
            star_meas = self.get_sensor_measurements()
            if np.isnan(star_meas).any():
                raise Exception('some stars are out of view from initial orientation')
        if angular_velocity is None:
            angvel = 0.1 * self.rng.standard_normal(3)
        else:
            angvel = np.array([
                angular_velocity['x'],
                angular_velocity['y'],
                angular_velocity['z'],
            ])
        pybullet.resetBaseVelocity(self.robot_id,
                            linearVelocity=[0., 0., 0.],
                            angularVelocity=angvel)

        # Shooting star position, orientation, and velocity
        if self.shootingstar:
            self.place_shootingstar()
        
        # Update display
        self._update_display()

    def _update_display(self):
        # hack to get GUI to update on MacOS
        time.sleep(0.01)
        keys = pybullet.getKeyboardEvents()
    
    def run(
            self,
            controller,
            max_time=5.0,
            data_filename=None,
            video_filename=None,
            print_debug=False
        ):

        self.data = {
            't': [],
            'yaw': [],
            'pitch': [],
            'roll': [],
            'w_x': [],
            'w_y': [],
            'w_z': [],
            'front_torque': [],
            'back_torque': [],
            'left_torque': [],
            'right_torque': [],
            'front_torque_command': [],
            'back_torque_command': [],
            'left_torque_command': [],
            'right_torque_command': [],
            'front_velocity': [],
            'back_velocity': [],
            'left_velocity': [],
            'right_velocity': [],
            'star_meas': [],
        }
        self.variables_to_log = getattr(controller, 'variables_to_log', [])
        for key in self.variables_to_log:
            if key in self.data.keys():
                raise Exception(f'Trying to log duplicate variable {key} (choose a different name)')
            self.data[key] = []

        # Always start from zero time
        self.t = 0.
        self.time_step = 0
        self.max_time_steps = int(max_time / self.dt)
        self.start_time = time.time()

        if video_filename is not None:
            # Import imageio
            imageio = importlib.import_module('imageio')

            # Open video
            fps = int(1 / self.dt)
            if print_debug:
                print(f'Creating a video with name {video_filename} and fps {fps}')
            w = imageio.get_writer(video_filename,
                                   format='FFMPEG',
                                   mode='I',
                                   fps=fps)

            # Add first frame to video
            rgba = self.snapshot()
            w.append_data(rgba)

        while True:
            all_done = self.step(controller)

            if video_filename is not None:
                if self.time_step % 25 == 0:
                    if print_debug:
                        print(f' {self.time_step} / {self.max_time_steps}')

                # Add frame to video
                rgba = self.snapshot()
                w.append_data(rgba)

            if all_done:
                break

            if (self.max_time_steps is not None) and (self.time_step == self.max_time_steps):
                break

        if video_filename is not None:
            # Close video
            w.close()

        if data_filename is not None:
            with open(data_filename, 'w') as f:
                json.dump(self.data, f)

        stop_time = time.time()
        stop_time_step = self.time_step

        elapsed_time = stop_time - self.start_time
        elapsed_time_steps = stop_time_step
        if (elapsed_time > 0) and print_debug:
            print(f'Simulated {elapsed_time_steps} time steps in {elapsed_time:.4f} seconds ({(elapsed_time_steps / elapsed_time):.4f} time steps per second)')

        # convert lists to numpy arrays
        data = self.data.copy()
        for key in data.keys():
            data[key] = np.array(data[key])

        return data

    def step(self, controller):
        # Get the current time
        self.t = self.time_step * self.dt

        # Get the sensor measurements
        star_meas = self.get_sensor_measurements()

        # Stop if any star is out of view
        if np.isnan(star_meas).any():
            return True

        # Get the state
        rpy, angvel, v = self.get_state()

        # Stop if any wheel exceeds maximum velocity
        if (np.abs(v) > self.v_max).any():
            return True

        # Get torque commands (run the controller)
        (
            front_torque_command,
            back_torque_command,
            left_torque_command,
            right_torque_command,
         ) = controller.run(self.t, star_meas)

        # Apply the torque commands
        (
            front_torque,
            back_torque,
            left_torque,
            right_torque,
        ) = self.set_actuator_commands(
            front_torque_command,
            back_torque_command,
            left_torque_command,
            right_torque_command,
        )

        # Apply external force to keep spacecraft position fixed
        pos, ori = pybullet.getBasePositionAndOrientation(self.robot_id)
        vel = pybullet.getBaseVelocity(self.robot_id)
        f = - 150. * np.array(pos) - 50. * np.array(vel[0])
        pybullet.applyExternalForce(
            self.robot_id,
            -1,
            f,
            pos,
            pybullet.WORLD_FRAME,
        )

        # Reset shooting star (if necessary)
        if self.shootingstar:
            pos, ori = pybullet.getBasePositionAndOrientation(self.shot_id)
            if np.linalg.norm(np.array(pos)) > 15:
                self.place_shootingstar()

        # Log data
        self.data['t'].append(self.t)
        self.data['yaw'].append(rpy[2])
        self.data['pitch'].append(rpy[1])
        self.data['roll'].append(rpy[0])
        self.data['w_x'].append(angvel[0])
        self.data['w_y'].append(angvel[1])
        self.data['w_z'].append(angvel[2])
        self.data['front_torque'].append(front_torque)
        self.data['back_torque'].append(back_torque)
        self.data['left_torque'].append(left_torque)
        self.data['right_torque'].append(right_torque)
        self.data['front_torque_command'].append(front_torque_command)
        self.data['back_torque_command'].append(back_torque_command)
        self.data['left_torque_command'].append(left_torque_command)
        self.data['right_torque_command'].append(right_torque_command)
        self.data['front_velocity'].append(v[0])
        self.data['back_velocity'].append(v[1])
        self.data['left_velocity'].append(v[2])
        self.data['right_velocity'].append(v[3])
        self.data['star_meas'].append(star_meas)
        for key in self.variables_to_log:
            val = getattr(controller, key, np.nan)
            if not np.isscalar(val):
                val = val.flatten().tolist()
            self.data[key].append(val)

        # Try to stay real-time
        if self.display:
            t = self.start_time + (self.dt * (self.time_step + 1))
            time_to_wait = t - time.time()
            while time_to_wait > 0:
                time.sleep(0.9 * time_to_wait)
                time_to_wait = t - time.time()

        # Take a simulation step
        pybullet.stepSimulation()

        # Increment time step
        self.time_step += 1

        return False

    def snapshot(self):
        # scope view
        pos, ori = pybullet.getBasePositionAndOrientation(self.robot_id)
        o_body_in_world = np.reshape(np.array(pos), (3, 1))
        R_body_in_world = np.reshape(np.array(pybullet.getMatrixFromQuaternion(ori)), (3, 3))
        p_eye = o_body_in_world
        p_target = (o_body_in_world + R_body_in_world @ np.array([[10.0], [0.], [0.]])).flatten()
        v_up = (R_body_in_world[:, 2]).flatten()
        view_matrix = pybullet.computeViewMatrix(p_eye, p_target, v_up)
        projection_matrix = pybullet.computeProjectionMatrixFOV(fov=45.0, aspect=1.0, nearVal=0.1, farVal=10.0)
        im = pybullet.getCameraImage(128, 128, viewMatrix=view_matrix, projectionMatrix=projection_matrix, renderer=pybullet.ER_BULLET_HARDWARE_OPENGL, shadow=0)
        rgba_scope = im[2]

        # hack to get black background color
        depth_scope = im[3]
        for i in range(3):
            rgba_scope[:, :, i] = np.where(depth_scope >= 0.99, 0., rgba_scope[:, :, i])

        # spacecraft view
        p_eye = 1.1 * np.array([-3., -4., 4.])
        p_target = np.array([0., 0., 0.])
        v_up = np.array([0., 0., 1.])
        view_matrix = pybullet.computeViewMatrix(p_eye, p_target, v_up)
        projection_matrix = pybullet.computeProjectionMatrixFOV(fov=60, aspect=1.0, nearVal=1.0, farVal=20.0)
        im = pybullet.getCameraImage(480, 480, viewMatrix=view_matrix, projectionMatrix=projection_matrix, renderer=pybullet.ER_BULLET_HARDWARE_OPENGL, shadow=1)
        rgba_world = im[2]

        # add "I" to scope view
        rgba_scope[40:42, 47:80, 0] = 255
        rgba_scope[40:42, 47:80, 1] = 255
        rgba_scope[40:42, 47:80, 2] = 255
        rgba_scope[87:89, 47:80, 0] = 255
        rgba_scope[87:89, 47:80, 1] = 255
        rgba_scope[87:89, 47:80, 2] = 255
        rgba_scope[41:87, 63:65, 0] = 255
        rgba_scope[41:87, 63:65, 1] = 255
        rgba_scope[41:87, 63:65, 2] = 255

        # hack to get black background color
        depth_world = im[3]
        for i in range(3):
            rgba_world[:, :, i] = np.where(depth_world >= 0.99, 0., rgba_world[:, :, i])

        # put scope view inside spacecraft view (picture-in-picture)
        rgba_world[10:138, 10:138, :] = rgba_scope

        return rgba_world
