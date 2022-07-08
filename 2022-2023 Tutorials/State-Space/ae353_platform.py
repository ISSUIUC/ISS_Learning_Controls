import numpy as np
import pybullet
import time
import os
import json
import importlib

class Simulator:
    def __init__(
                self,
                display=True,
                seed=None,
                width=640,
                height=480,
                roll=0.,
                damping=0.,
                tau_max=5.,
            ):

        # Random number generator
        self.rng = np.random.default_rng(seed)

        # Size of display
        self.width = width
        self.height = height

        # Time step
        self.dt = 0.01

        # Other parameters
        self.roll = roll
        self.damping = damping
        self.tau_max = tau_max

        # Connect to and configure pybullet
        self.display = display
        if self.display:
            pybullet.connect(pybullet.GUI, options=f'--width={width} --height={height}')
            pybullet.configureDebugVisualizer(
                pybullet.COV_ENABLE_GUI, 0,
                lightPosition=[10., 10., 10.],
            )
        else:
            pybullet.connect(pybullet.DIRECT)
        pybullet.setGravity(0, 0, -9.81)
        pybullet.setPhysicsEngineParameter(
            fixedTimeStep=self.dt,
            numSubSteps=4,
            restitutionVelocityThreshold=0.05,
            enableFileCaching=0,
        )

        # Load plane
        self.plane_id = pybullet.loadURDF(
            os.path.join('.', 'urdf', 'plane.urdf'),
            basePosition=np.array([0., 0., 0.]),
            baseOrientation=pybullet.getQuaternionFromEuler([0., self.roll, 0.]),
            useFixedBase=1,
        )
        
        # Load platform
        self.robot_id = pybullet.loadURDF(
            os.path.join('.', 'urdf', 'platform.urdf'),
            flags=(
                    pybullet.URDF_USE_IMPLICIT_CYLINDER |
                    pybullet.URDF_USE_INERTIA_FROM_FILE
            ),
            basePosition=np.array([0., 0., 0.]),
            baseOrientation=pybullet.getQuaternionFromEuler([0., self.roll, 0.]),
        )
        
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
            'base_to_platform',
            'connector_to_wheel',
        ]
        self.num_joints = len(self.joint_names)
        self.joint_ids = np.array([self.joint_map[joint_name] for joint_name in self.joint_names])

        # Set damping of all joints to given value
        for id in self.joint_ids:
            pybullet.changeDynamics(self.robot_id, id, jointDamping=damping)

        # Disable velocity control on each joint so we can use torque control
        pybullet.setJointMotorControlArray(
            self.robot_id,
            self.joint_ids,
            pybullet.VELOCITY_CONTROL,
            forces=np.zeros(self.num_joints)
        )

        # Eliminate linear and angular damping (a poor model of drag)
        pybullet.changeDynamics(self.robot_id, -1, linearDamping=0., angularDamping=0.)
        for joint_id in range(pybullet.getNumJoints(self.robot_id)):
            pybullet.changeDynamics(self.robot_id, joint_id, linearDamping=0., angularDamping=0.)
        
        # Camera view
        self.camera_sideview()
    
    def set_roll(self, roll):
        self.roll = roll
        pybullet.resetBasePositionAndOrientation(
            self.plane_id,
            np.array([0., 0., 0.]),
            pybullet.getQuaternionFromEuler([0., self.roll, 0.]),
        )
        pybullet.resetBasePositionAndOrientation(
            self.robot_id,
            np.array([0., 0., 0.]),
            pybullet.getQuaternionFromEuler([0., self.roll, 0.]),
        )
        self.reset()

    def get_sensor_measurements(self):
        joint_states = pybullet.getJointStates(self.robot_id, self.joint_ids)
        platform_angle = joint_states[0][0]
        platform_velocity = joint_states[0][1]
        wheel_angle = joint_states[1][0]
        wheel_velocity = joint_states[1][1]
        return platform_angle, platform_velocity, wheel_angle, wheel_velocity

    def set_actuator_commands(self, wheel_torque_command):
        assert(np.isscalar(wheel_torque_command))
        wheel_torque = np.clip(wheel_torque_command, -self.tau_max, self.tau_max)
        self.set_joint_torque(np.array([0., wheel_torque]))
        return wheel_torque
    
    def set_joint_torque(self, tau):
        assert(tau.shape[0] == self.num_joints)
        zero_gains = tau.shape[0] * (0.,)
        pybullet.setJointMotorControlArray(
            self.robot_id,
            self.joint_ids,
            pybullet.TORQUE_CONTROL,
            forces=tau,
            positionGains=zero_gains,
            velocityGains=zero_gains
        )
    
    def reset(
            self,
            platform_angle=0.,
            platform_velocity=0.,
            wheel_angle=0.,
            wheel_velocity=0.,
        ):
        
        pybullet.resetJointState(
            self.robot_id,
            self.joint_map['base_to_platform'],
            platform_angle,
            platform_velocity,
        )

        pybullet.resetJointState(
            self.robot_id,
            self.joint_map['connector_to_wheel'],
            wheel_angle,
            wheel_velocity,
        )

        self.camera()

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
            'platform_angle': [],
            'platform_velocity': [],
            'wheel_angle': [],
            'wheel_velocity': [],
            'wheel_torque': [],
            'wheel_torque_command': [],
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
                if self.time_step % 100 == 0:
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
        # Never stop early
        all_done = False

        # Get the current time
        self.t = self.time_step * self.dt

        # Get the sensor measurements
        platform_angle, platform_velocity, wheel_angle, wheel_velocity = self.get_sensor_measurements()

        # Get the torque command (run the controller)
        wheel_torque_command = controller.run(
            self.t,
            platform_angle,
            platform_velocity,
            wheel_angle,
            wheel_velocity,
        )

        # Apply the torque command
        wheel_torque = self.set_actuator_commands(
            wheel_torque_command,
        )

        # Log data
        self.data['t'].append(self.t)
        self.data['platform_angle'].append(platform_angle)
        self.data['platform_velocity'].append(platform_velocity)
        self.data['wheel_angle'].append(wheel_angle)
        self.data['wheel_velocity'].append(wheel_velocity)
        self.data['wheel_torque'].append(wheel_torque)
        self.data['wheel_torque_command'].append(wheel_torque_command)
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

        return all_done

    def snapshot(self):
        pos = self.camera_target
        yaw = -90 + self.camera_yaw
        aspect = self.width / self.height
        view_matrix = pybullet.computeViewMatrixFromYawPitchRoll(pos, self.camera_distance, yaw, -self.camera_pitch, 0., 2)
        projection_matrix = pybullet.computeProjectionMatrixFOV(fov=90, aspect=aspect, nearVal=0.01, farVal=100.0)
        im = pybullet.getCameraImage(
            self.width, self.height,
            viewMatrix=view_matrix,
            projectionMatrix=projection_matrix,
            renderer=pybullet.ER_BULLET_HARDWARE_OPENGL,
            shadow=1,
            lightDirection=[10., 10., 10.],
        )
        rgba = im[2]
        return rgba

    def camera(self):
        if self.display:
            pybullet.resetDebugVisualizerCamera(self.camera_distance, -90 + self.camera_yaw, -self.camera_pitch, self.camera_target)

            # hack to get GUI to update on MacOS
            time.sleep(0.01)
            keys = pybullet.getKeyboardEvents()

    def camera_topview(self):
        self.camera_target = np.array([0.0, 0.0, 0.0])
        self.camera_distance = 4.
        self.camera_pitch = np.clip(90. - np.rad2deg(self.roll), -89.9, 89.9)
        self.camera_yaw = 180.
        self.camera()

    def camera_sideview(self):
        self.camera_target = np.array([0.0, 0.0, 0.0])
        self.camera_distance = 3.
        self.camera_pitch = 30.
        self.camera_yaw = 150.
        self.camera()
