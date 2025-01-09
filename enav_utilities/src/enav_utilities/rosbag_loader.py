#!/usr/bin/env python

""" rosbag_data_load.py

    Utility functions to load data from the energy-aware planetary navigation dataset

    Authors:    Olivier Lamarre <olivier.lamarre@robotics.utias.utoronto.ca>
                Oliver Limoyo <oliver.limoyo@robotics.utias.utoronto.ca>
    Affl.:      Space and Terrestrial Autonomous Robotic Systems Laboratory
                University of Toronto
    Date:       January 27, 2019
"""

import sys
# import time
import itertools

import numpy as np
import rosbag
import utm

import sensor_msgs.point_cloud2 as pc2


class ENAVRosbagLoader:

    def __init__(self, filename):

        # Initialize object by providing bag file path
        self.file = filename
        self.bag = rosbag.Bag(filename, 'r')

        # Default topic names
        self.tpc_names = {
            "power" : "/status",
            "imu" : "/imu",
            "pyranometer" : "/pyranometer",
            "gps" : "/gps",
            "pancam" : "/omni_stitched_image",
            "monocam" : "/mono_image",
            "husky_encoder": "/joint_states",
            "husky_odometry": "/husky_velocity_estimate",
            "husky_cmd_vel": "/husky_commanded_velocity",
            "pointclouds": "/omni_stitched_cloud",
            "pose_estimates": "/global_odometry_utm",
            "relative_sun_orientation": "/relative_sun_orientation",
            "global_sun_orientation": "/global_sun_orientation",
        }

        # Data retrieval status
        self.status = 0

    def load_cmd_vel_data(self, rel_time=False):
        """ Loads commanded velocities to the Husky rover base

        Input:  rel_time - if true, returned time is the number of seconds from
            the start of the bag (as opposed to an absolute timestamp)

        Return: <float> numpy array of 7 columns:
                time [s],
                x_linear_velocity [m/s],
                y_linear_velocity [m/s],
                z_linear_velocity [m/s],
                x_angular_velocity [rad/s],
                y_angular_velocity [rad/s],
                z_angular_velocity [rad/s],
        """

        tot_msg_count = self.bag.get_message_count(self.tpc_names["husky_cmd_vel"])

        data = np.empty((0,7), np.float)

        valid_msg_count = 0

        print("Retrieving husky_cmd_vel data from {} ...".format(self.file))
        print("Number of husky_cmd_vel messages: {}".format(tot_msg_count))

        time_offset = self.bag.get_start_time() if rel_time else 0

        for _, msg, time in self.bag.read_messages(self.tpc_names["husky_cmd_vel"]):

            curr_time = time.to_sec() - time_offset
            temp = np.array([curr_time,
                            msg.linear.x,
                            msg.linear.y,
                            msg.linear.z,
                            msg.angular.x,
                            msg.angular.y,
                            msg.angular.z])

            # Populate main data array
            data = np.vstack([data,temp])

            # Show process status:
            valid_msg_count +=1
            self.status = round(100*float(valid_msg_count)/tot_msg_count)

            sys.stdout.write('\r')
            sys.stdout.write("Progress: {} %".format(self.status))
            sys.stdout.flush()

        sys.stdout.write("\n")

        return data

    def load_est_vel_data(self, rel_time=False):
        """ Loads odometry data from the Husky computed from wheel encoders.

        Input:  rel_time - if true, returned time is the number of seconds from
            the start of the bag (as opposed to an absolute timestamp)

        Return: <float> numpy array of 7 columns:
                time [s],
                x_linear_velocity [m/s],
                y_linear_velocity [m/s],
                z_linear_velocity [m/s],
                x_angular_velocity [rad/s],
                y_angular_velocity [rad/s],
                z_angular_velocity [rad/s],
        """

        tot_msg_count = self.bag.get_message_count(self.tpc_names["husky_odometry"])

        data = np.empty((0,7), np.float)

        valid_msg_count = 0

        print("Retrieving husky_odometry data from {} ...".format(self.file))
        print("Number of husky_odometry messages: {}".format(tot_msg_count))

        time_offset = self.bag.get_start_time() if rel_time else 0

        for _, msg, time in self.bag.read_messages(self.tpc_names["husky_odometry"]):

            curr_time = time.to_sec() - time_offset
            temp = np.array([curr_time,
                            msg.twist.twist.linear.x,
                            msg.twist.twist.linear.y,
                            msg.twist.twist.linear.z,
                            msg.twist.twist.angular.x,
                            msg.twist.twist.angular.y,
                            msg.twist.twist.angular.z])

            # Populate main data array
            data = np.vstack([data,temp])

            # Show process status:
            valid_msg_count +=1
            self.status = round(100*float(valid_msg_count)/tot_msg_count)

            sys.stdout.write('\r')
            sys.stdout.write("Progress: {} %".format(self.status))
            sys.stdout.flush()

        sys.stdout.write("\n")

        return data

    def load_encoder_data(self, rel_time=False):
        """ Loads encoder data from the Husky.

        Input:  rel_time - if true, returned time is the number of seconds from
            the start of the bag (as opposed to an absolute timestamp)

        Return: <float> numpy array of 9 columns:
                time [s],
                front_left_wheel position [rad],
                front_right_wheel position [rad],
                rear_left_wheel position [rad],
                rear_right_wheel position [rad],
                front_left_wheel velocity [rad/s],
                front_right_wheel velocity [rad/s],
                rear_left_wheel velocity [rad/s],
                rear_right_wheel velocity [rad/s],
        """

        tot_msg_count = self.bag.get_message_count(self.tpc_names["husky_encoder"])

        data = np.empty((0,9), np.float)

        valid_msg_count = 0

        print("Retrieving husky_encoder data from {} ...".format(self.file))
        print("Number of husky_encoder messages: {}".format(tot_msg_count))

        time_offset = self.bag.get_start_time() if rel_time else 0

        for _, msg, time in self.bag.read_messages(self.tpc_names["husky_encoder"]):

            curr_time = time.to_sec() - time_offset

            pos = np.asarray(msg.position)
            vel = np.asarray(msg.velocity)
            enc = np.hstack((curr_time, pos,vel))

            # Populate main data array
            data = np.vstack([data,enc])

            # Show process status:
            valid_msg_count +=1
            self.status = round(100*float(valid_msg_count)/tot_msg_count)

            sys.stdout.write('\r')
            sys.stdout.write("Progress: {} %".format(self.status))
            sys.stdout.flush()

        sys.stdout.write("\n")

        return data

    def load_irradiance_data(self, rel_time=False):
        """ Loads solar irradiance data from the pyranometer

        Input:  rel_time - if true, returned time is the number of seconds from
            the start of the bag (as opposed to an absolute timestamp)

        Return: <float> numpy array with 3 columns:
                timestamp [s],
                measured solar irradiance [W/m^2],
                clear-sky solar irradiance [W/m^2]
        """

        tot_msg_count = self.bag.get_message_count(self.tpc_names["pyranometer"])

        data = np.empty((0,3), np.float)

        valid_msg_count = 0

        print("Retrieving pyranometer data from {} ...".format(self.file))
        print("Number of pyranometer messages: {}".format(tot_msg_count))

        time_offset = self.bag.get_start_time() if rel_time else 0

        for _, msg, time in self.bag.read_messages(self.tpc_names["pyranometer"]):

            curr_time = time.to_sec() - time_offset
            temp = np.array([curr_time,
                                msg.data,
                                msg.theoretical_clearsky_horizontal])

            # Populate main data array
            data = np.vstack([data,temp])

            # Show process status:
            valid_msg_count +=1
            self.status = round(100*float(valid_msg_count)/tot_msg_count)

            sys.stdout.write('\r')
            sys.stdout.write("Progress: {} %".format(self.status))
            sys.stdout.flush()

        sys.stdout.write("\n")

        return data

    def load_energy_data(self, rel_time=False):
        """ Loads data from driving motors power monitors onboard the Husky
            Energy values are obtained by integrating power consumption values

        Input:  rel_time - if true, returned time is the number of seconds from
            the start of the bag (as opposed to an absolute timestamp)

        Return: <float> numpy array of 9 columns:
                timestamp [s],
                l_motor_volt [V],
                l_motor_curr [A],
                r_motor_volt [V],
                r_motor_curr [A],
                l_motor_power [W],
                r_motor_power [W],
                l_motor_cummul_energy [J],
                r_motor_cummul_energy [J]
        """

        tot_msg_count = self.bag.get_message_count(self.tpc_names["power"])

        data = np.empty((0,5), np.float)

        valid_msg_count = 0

        print("Retrieving power data from {} ...".format(self.file))
        print("Number of power messages: {}".format(tot_msg_count))

        time_offset = self.bag.get_start_time() if rel_time else 0

        for _, msg, time in self.bag.read_messages(self.tpc_names["power"]):

            curr_time = time.to_sec() - time_offset
            temp = np.array([curr_time,
                                msg.left_driver_voltage,
                                msg.left_driver_current,
                                msg.right_driver_voltage,
                                msg.right_driver_current])

            # Populate main data array
            data = np.vstack([data,temp])

            # Show process status:
            valid_msg_count +=1
            self.status = round(100*float(valid_msg_count)/tot_msg_count)

            sys.stdout.write('\r')
            sys.stdout.write("Progress: {} %".format(self.status))
            sys.stdout.flush()

        sys.stdout.write("\n")

        # Compute motor powers and append to main data array
        l_power = np.reshape(np.multiply(data[:,1],data[:,2]), (-1,1))
        r_power = np.reshape(np.multiply(data[:,3],data[:,4]), (-1,1))

        data = np.hstack([data, l_power])
        data = np.hstack([data, r_power])

        # Compute motor energies and append to main data array
        l_energy = self.energy_from_power(data[:,0], data[:,5])
        l_energy = np.reshape(l_energy, (-1,1))

        r_energy = self.energy_from_power(data[:,0], data[:,6])
        r_energy = np.reshape(r_energy, (-1,1))

        data = np.hstack([data, l_energy])
        data = np.hstack([data, r_energy])

        return data

    def load_imu_data(self, rel_time=False):
        """ Loads IMU data

        Input:  rel_time - if true, returned time is the number of seconds from
            the start of the bag (as opposed to an absolute timestamp)

        Return: <float> numpy array of 11 columns:
                timestamp [s],
                x_linear_acceleration [m/s^2],
                y_linear_acceleration [m/s^2],
                z_linear_acceleration [m/s^2],
                x_angular_velocity [rad/s],
                y_angular_velocity [rad/s],
                z_angular_velocity [rad/s],
                x_orientation,
                y_orientation,
                z_orientation,
                w_orientation
        """

        tot_msg_count = self.bag.get_message_count(self.tpc_names["imu"])

        data = np.empty((0,11), np.float)

        valid_msg_count = 0

        print("Retrieving IMU data from {} ...".format(self.file))
        print("Number of IMU messages: {}".format(tot_msg_count))

        time_offset = self.bag.get_start_time() if rel_time else 0

        for _, msg, time in self.bag.read_messages(self.tpc_names["imu"]):

            curr_time = time.to_sec() - time_offset
            temp = np.array([curr_time,
                                msg.linear_acceleration.x,
                                msg.linear_acceleration.y,
                                msg.linear_acceleration.z,
                                msg.angular_velocity.x,
                                msg.angular_velocity.y,
                                msg.angular_velocity.z,
                                msg.orientation.x,
                                msg.orientation.y,
                                msg.orientation.z,
                                msg.orientation.w])

            # Populate main data array
            data = np.vstack([data,temp])

            # Show process status:
            valid_msg_count +=1
            self.status = round(100*float(valid_msg_count)/tot_msg_count)

            sys.stdout.write('\r')
            sys.stdout.write("Progress: {} %".format(self.status))
            sys.stdout.flush()

        sys.stdout.write("\n")

        return data

    def load_pointcloud_data(
            self,
            pc_source="omni_stitched_cloud",
            time_range=None,
            rel_time=False
        ):
        """ Loads point cloud data from stereo omnidirectional camera

        Input:  pc_source - point cloud source, from {omni_cloud0, ...,
                    omni_cloud4, omni_stitched_cloud}
                time_range - Load clouds inside of this time range (2-tuple of
                    floating point timestamps (seconds)). If rel_time is True,
                    these time values are expected to be times elapsed from the
                    start of the bag, in seconds. Absolute ROS times otherwise.
                rel_time - if true, returned time is the number of seconds from
                    the start of the bag (as opposed to an absolute timestamp)

        Return: tuple of (times [s], clouds).
                times is a <float> numpy array of dimension: (batch,)
                clouds is a list of <float> numpy arrays. Each <float> numpy
                array (a single pointcloud) is of dimension: (n_points, 3),
                where each column corresponds to x,y,z, respectively.
        """

        if pc_source == "omni_stitched_cloud":
            pc_gen = self.bag.read_messages(self.tpc_names["pointclouds"])
            tot_msg_count = self.bag.get_message_count(self.tpc_names["pointclouds"])
        elif pc_source in ["omni_cloud{}".format(x) for x in range(10)]:
            idx = ["omni_cloud{}".format(x) for x in range(10)].index(pc_source)
            pc_gen = self.bag.read_messages("omni_cloud{}".format(idx))
            tot_msg_count = self.bag.get_message_count("omni_cloud{}".format(idx))

        valid_msg_count = 0

        print("Retrieving pointcloud data from {} ...").format(self.file)
        print("Number of pointcloud messages: {}").format(tot_msg_count)

        if time_range == None:
            print("Loading all {} pointclouds".format(tot_msg_count))
            pc_count = tot_msg_count

        elif type(time_range) is tuple:

            if rel_time:
                abs_time_range = tuple((t+self.bag.get_start_time() for t in time_range))
            else:
                abs_time_range = time_range

            t_range = self.time_to_timestep(abs_time_range, tot_msg_count)

            # Quick check for tuple range
            if not (0 <= t_range[0] < tot_msg_count and 0 <= t_range[1] < tot_msg_count):
                raise ValueError("Invalid timestep range {} for pointcloud dataset of size {}" \
                                .format(t_range, tot_msg_count))

            print("Loading pointclouds from timesteps {} to {}".format(t_range[0], t_range[1]))
            # Don't load all the data, instead slice the generator at the desired range
            pc_gen = itertools.islice(pc_gen, t_range[0], t_range[1])
            pc_count = (t_range[1] - t_range[0]) + 1

        else:
            raise ValueError("Invalid type {} for 't_range', t_range is a tuple (start,end) or None" % type(t_range))

        time_offset = self.bag.get_start_time() if rel_time else 0

        time_data = np.empty((pc_count))
        data = []
        for i, (_, msg, time) in enumerate(pc_gen):

            time_data[i] = time.to_sec() - time_offset

            # for PointCloud2
            pc = [[point[0], point[1], point[2]] for point in
                    pc2.read_points(msg, field_names = ("x", "y", "z"), skip_nans=True)]
            # for PointCloud
            # pc = [[point.x, point.y, point.z] for point in msg.points]

            data.append(np.asarray(pc, dtype=np.float32))
            valid_msg_count +=1

        return (time_data, data)

    def load_image_data(self, img_source="mono_image", time_range=None, rel_time=False):
        """ Loads image data from a specific camera

        Input:  string img_source - get images from {"omni_image0", ..., "omni_image9",
                "omni_stitched_image", "omni_stitched_disparity", "mono_image"}
                time_range - Load images inside of this time range (2-tuple of
                    floating point timestamps (seconds)). If rel_time is True,
                    these time values are expected to be times elapsed from the
                    start of the bag, in seconds. Absolute ROS times otherwise.
                rel_time - if true, returned time is the number of seconds from
                    the start of the bag (as opposed to an absolute timestamp)

        Return: tuple of (time [s], images).
                time is a <float> numpy array of dimension: (batch,).
                images is a <int> numpy array of dimension:
                (batch, height, width) or (batch , height, width, channel).
        """

        # Check which camera to grab images from
        if img_source == "mono_image":
            img_gen = self.bag.read_messages(self.tpc_names["monocam"])
            tot_msg_count = self.bag.get_message_count(self.tpc_names["monocam"])
            image_type = "grayscale"
        elif img_source == "omni_stitched_image":
            img_gen = self.bag.read_messages(self.tpc_names["pancam"])
            tot_msg_count = self.bag.get_message_count(self.tpc_names["pancam"])
            image_type = "rgb"
        elif img_source in ["omni_image{}".format(x) for x in range(10)]:
            idx = ["omni_image{}".format(x) for x in range(10)].index(img_source)
            img_gen = self.bag.read_messages("/omni_image{}".format(idx))
            tot_msg_count = self.bag.get_message_count("/omni_image{}".format(idx))
            image_type = "rgb"
        elif img_source == "omni_stitched_disparity":
            img_gen = self.bag.read_messages("/omni_stitched_disparity")
            tot_msg_count = self.bag.get_message_count("/omni_stitched_image")
            image_type = "disp"
        else:
            raise ValueError("{} is not from {occam_image0, ..., occam_image9, omni_stitched_image, omni_stitched_disparity, mono_image}".format(img_source))

        valid_msg_count = 0

        print("Retrieving '{}' image data from {} ...".format(img_source, self.file))
        print("Number of '{}' image messages: {}".format(img_source, tot_msg_count))

        if time_range == None:
            print("Loading all {} images".format(tot_msg_count))

            # Calculate the amount of images to pre-allocate later
            img_count = tot_msg_count
        elif type(time_range) is tuple:

            if rel_time:
                abs_time_range = tuple((t+self.bag.get_start_time() for t in time_range))
            else:
                abs_time_range = time_range

            t_range = self.time_to_timestep(abs_time_range, tot_msg_count)

            # Quick check for tuple range
            if not (0 <= t_range[0] < tot_msg_count and 0 <= t_range[1] < tot_msg_count):
                raise ValueError(
                    "Invalid timestep range {} for image dataset of size {}. " \
                                .format(t_range, tot_msg_count) +\
                    "Current rosbag start-end timestamps: {}-{}".format(self.bag.get_start_time(),self.bag.get_end_time())
                )

            print("Loading images from timesteps {} to {}".format(t_range[0], t_range[1]))
            # Don't load all the data, instead slice the generator at the desired range
            img_gen = itertools.islice(img_gen, t_range[0], t_range[1])

            # Calculate the amount of images to pre-allocate later
            img_count = (t_range[1] - t_range[0]) + 1
        else:
            raise ValueError("Invalid type {} for 't_range', t_range is a tuple (start,end) or None" % type(t_range))

        time_offset = self.bag.get_start_time() if rel_time else 0

        time_data = np.empty((img_count))
        for i, (_, msg, time) in enumerate(img_gen):
            img = np.fromstring(msg.data, dtype=np.uint8)

            time_data[i] = time.to_sec() - time_offset

            if i == 0:
                # Get the dimensions from the first image and pre-allocate
                w = msg.width
                h = msg.height
                if image_type == "grayscale":
                    data = np.empty((img_count,h,w), np.uint8)
                elif image_type == "disp":
                    data = np.empty((img_count,h,w,3), np.uint8)
                else:
                    data = np.empty((img_count,h,w,3), np.uint8)

            if image_type == "grayscale":
                img = img.reshape(h,w)
                data[i,:,:] = img
            elif image_type == "disp":
                img = img.reshape(h,w, 3)
                data[i,:,:] = img
            else:
                img = img.reshape(h,w, 3)
                data[i,:,:,:] = img

            valid_msg_count +=1

        return (time_data, data)

    def energy_from_power(self, time, power):
        """ Compute cummulative energy consumed by integrating power values

        Input:  time - (batch,) time array [s]
                power - (batch,) power array [W]

        Return: energy - (batch,) energy array [J]
        """

        n = time.shape[0]
        energy = np.zeros(n)

        for i in range(n-1):
            energy[i+1] = energy[i] + np.trapz(power[i:i+2], x=time[i:i+2])

        return energy

    def load_gps_data(self, ret_utm=False, rel_time=False):
        """ Loads GPS data

        Input:  ret_utm - return the position as UTM coords (instead of lat-lon)
                rel_time - if true, returned time is the number of seconds from
                    the start of the bag (as opposed to an absolute timestamp)

        Return: <float> numpy array of 4 columns if ret_utm argument is False:
                ROS timestamp [s],
                latitude [deg, +ve = Northern hemisphere]
                longitude [deg, +ve = East of Prime Meridian]
                altitude [m]

                OR

                <float> numpy array of 4 columns if ret_utm argument is True:
                ROS timestamp [s],
                UTM easting [m, +ve = towards East]
                UTM northing [m, +ve = towards North]
                altitude [m]
        """

        tot_msg_count = self.bag.get_message_count(self.tpc_names["gps"])

        data = np.empty((0,4), np.float)

        valid_msg_count = 0

        print("Retrieving GPS data from {} ...".format(self.file))
        print("Number of GPS messages: {}".format(tot_msg_count))

        time_offset = self.bag.get_start_time() if rel_time else 0

        for _, msg, time in self.bag.read_messages(self.tpc_names["gps"]):

            curr_time = time.to_sec() - time_offset
            lat = msg.latitude
            lon = msg.longitude
            alt = msg.altitude

            # Retrieve current data
            if ret_utm:
                easting, northing = utm.from_latlon(lat, lon)[0:2]
                temp = np.array([curr_time, easting, northing, alt])
            else:
                temp = np.array([curr_time, lat, lon, alt])

            # Populate main data array
            data = np.vstack([data,temp])

            # Show process status:
            valid_msg_count +=1
            self.status = round(100*float(valid_msg_count)/tot_msg_count)

            sys.stdout.write('\r')
            sys.stdout.write("Progress: {} %".format(self.status))
            sys.stdout.flush()

        sys.stdout.write("\n")

        return data

    def load_VINS_data(self, rel_time=False):
        """ Loads pose estimates from VINS-Fusion

        Input:  rel_time - if true, returned time is the number of seconds from
                    the start of the bag (as opposed to an absolute timestamp)

        Return: <float> numpy array of 8 columns:
                time [s],
                x_linear_position [m],
                y_linear_position [m],
                z_linear_position [m],
                x_orientation,
                y_orientation,
                z_orientation,
                w_orientation
        """

        tot_msg_count = self.bag.get_message_count(self.tpc_names["pose_estimates"])

        data = np.empty((0,8), np.float)

        valid_msg_count = 0

        print("Retrieving VINS pose estimate data from {} ...".format(self.file))
        print("Number of VINS pose estimate messages: {}".format(tot_msg_count))

        time_offset = self.bag.get_start_time() if rel_time else 0

        # Retrieve latest path
        for _, msg, time in self.bag.read_messages(self.tpc_names["pose_estimates"]):

            curr_time = time.to_sec() - time_offset
            temp = np.array([curr_time,
                            msg.pose.pose.position.x,
                            msg.pose.pose.position.y,
                            msg.pose.pose.position.z,
                            msg.pose.pose.orientation.x,
                            msg.pose.pose.orientation.y,
                            msg.pose.pose.orientation.z,
                            msg.pose.pose.orientation.w])

            # Populate main data array
            data = np.vstack([data,temp])

            # Show process status:
            valid_msg_count +=1
            self.status = round(100*float(valid_msg_count)/tot_msg_count)

            sys.stdout.write('\r')
            sys.stdout.write("Progress: {} %".format(self.status))
            sys.stdout.flush()

        sys.stdout.write("\n")

        return data

    def load_sun_position_data(self, rel_time=False, reference="global"):
        """ Loads sun position data

        Input:  rel_time - if true, returned time is the number of seconds from
                    the start of the bag (as opposed to an absolute timestamp)
                reference - sun pose vector with respect to "global" or "rover" frame

        Return: <float> numpy array of 8 columns:
                time [s],
                x_linear_position [m],
                y_linear_position [m],
                z_linear_position [m],
                x_orientation,
                y_orientation,
                z_orientation,
                w_orientation
        """

        tot_msg_count = self.bag.get_message_count(self.tpc_names["relative_sun_orientation"])

        data = np.empty((0,8), np.float)

        valid_msg_count = 0

        print("Retrieving sun orientation data from {} ...".format(self.file))
        print("Number of sun orientation data messages: {}".format(tot_msg_count))

        if reference == "global":
            data_generator = self.bag.read_messages(self.tpc_names["global_sun_orientation"])
        elif reference == "relative":
            data_generator = self.bag.read_messages(self.tpc_names["relative_sun_orientation"])
        else:
            raise NotImplementedError

        time_offset = self.bag.get_start_time() if rel_time else 0

        # Retrieve latest path
        for _, msg, time in data_generator:

            curr_time = time.to_sec() - time_offset
            temp = np.array([curr_time,
                            msg.pose.position.x,
                            msg.pose.position.y,
                            msg.pose.position.z,
                            msg.pose.orientation.x,
                            msg.pose.orientation.y,
                            msg.pose.orientation.z,
                            msg.pose.orientation.w])

            # Populate main data array
            data = np.vstack([data,temp])

            # Show process status:
            valid_msg_count +=1
            self.status = round(100*float(valid_msg_count)/tot_msg_count)

            sys.stdout.write('\r')
            sys.stdout.write("Progress: {} %".format(self.status))
            sys.stdout.flush()

        sys.stdout.write("\n")

        return data

    def time_to_timestep(self, time_range, tot_msg_count):
        percentage_range_start = (time_range[0] - self.bag.get_start_time())/(self.bag.get_end_time()-self.bag.get_start_time())
        percentage_range_end = (time_range[1] - self.bag.get_start_time())/(self.bag.get_end_time()-self.bag.get_start_time())

        t_range_start = int(tot_msg_count*percentage_range_start)
        t_range_end = int(tot_msg_count*percentage_range_end)
        return (t_range_start, t_range_end)