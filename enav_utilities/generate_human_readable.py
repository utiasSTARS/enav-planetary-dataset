#!/usr/bin/env python
import argparse
import os, sys, inspect
from rosbag_data_load import FetchEnergyDataset
import numpy as np
import argparse

def write(data, header, filename):
    assert data.shape[1] == len(header.split(','))
    np.savetxt(filename, data, delimiter=",", header=header)  

def main():
    parser = argparse.ArgumentParser(description=".")
    parser.add_argument("--bag_file", help="Input ROS bag.")
    parser.add_argument("--output_dir", help="Output directory.")
    args = parser.parse_args()

    bag_obj = FetchEnergyDataset(args.bag_file)
    
    pyra_filename = args.output_dir + "/pyranometer.txt"
    pyra_data = bag_obj.load_irradiance_data(rel_time=False)
    pyra_header = "time [s],\
                    measured solar irradiance [W/m^2],\
                    clear-sky solar irradiance [W/m^2]"
    write(pyra_data, pyra_header, pyra_filename)

    # energy_filename = args.output_dir + "/energy.txt"
    # energy_data = bag_obj.load_energy_data(rel_time=False)
    # energy_header = "time [s],\
    #                 l_motor_volt [V],\
    #                 l_motor_curr [A],\
    #                 r_motor_volt [V],\
    #                 r_motor_curr [A],\
    #                 l_motor_power [W],\
    #                 r_motor_power [W],\
    #                 l_motor_cummul_energy [J],\
    #                 r_motor_cummul_energy [J]"
    # write(energy_data, energy_header, energy_filename)

    # odom_filename = args.output_dir + "/velocity-estimates.txt"
    # odom_data = bag_obj.load_est_vel_data(rel_time=False)
    # odom_header = "time [s],\
    #                 x_linear_velocity [m/s],\
    #                 y_linear_velocity [m/s],\
    #                 z_linear_velocity [m/s],\
    #                 x_angular_velocity [rad/s],\
    #                 y_angular_velocity [rad/s],\
    #                 z_angular_velocity [rad/s]"
    # write(odom_data, odom_header, odom_filename)

    # cmd_vel_filename = args.output_dir + "/cmd-velocities.txt"
    # cmd_vel_data = bag_obj.load_cmd_vel_data(rel_time=False)
    # cmd_vel_header = "time [s], \
    #                     x_linear_velocity [m/s],\
    #                     y_linear_velocity [m/s],\
    #                     z_linear_velocity [m/s],\
    #                     x_angular_velocity [rad/s],\
    #                     y_angular_velocity [rad/s],\
    #                     z_angular_velocity [rad/s]"
    # write(cmd_vel_data, cmd_vel_header, cmd_vel_filename)

    # enc_filename = args.output_dir + "/wheel-encoder.txt"
    # enc_data = bag_obj.load_encoder_data(rel_time=False)
    # enc_header = "time [s],\
    #                 front_left_wheel position [rad],\
    #                 front_right_wheel position [rad],\
    #                 rear_left_wheel position [rad],\
    #                 rear_right_wheel position [rad],\
    #                 front_left_wheel velocity [rad/s],\
    #                 front_right_wheel velocity [rad/s],\
    #                 rear_left_wheel velocity [rad/s],\
    #                 rear_right_wheel velocity [rad/s]"
    # write(enc_data, enc_header, enc_filename)

    # imu_filename = args.output_dir + "/imu.txt"
    # imu_data = bag_obj.load_imu_data(rel_time=False)
    # imu_header = "time [s],\
    #                 x_linear_acceleration [m/s^2],\
    #                 y_linear_acceleration [m/s^2],\
    #                 z_linear_acceleration [m/s^2],\
    #                 x_angular_velocity [rad/s],\
    #                 y_angular_velocity [rad/s],\
    #                 z_angular_velocity [rad/s],\
    #                 x_orientation,\
    #                 y_orientation,\
    #                 z_orientation,\
    #                 w_orientation"
    # write(imu_data, imu_header, imu_filename)

    VINS_filename = args.output_dir + "/global-pose-utm.txt"
    VINS_data = bag_obj.load_VINS_data(rel_time=False)
    VINS_header = "time [s],\
                    x_linear_position [m],\
                    y_linear_position [m],\
                    z_linear_position [m],\
                    x_orientation,\
                    y_orientation,\
                    z_orientation,\
                    w_orientation"
    write(VINS_data, VINS_header, VINS_filename)

    # gps_filename = args.output_dir + "/gps-latlong.txt"
    # gps_data = bag_obj.load_gps_data(rel_time=False)
    # gps_header = "time [s],\
    #                 latitude [deg, +ve = towards North]\
    #                 longitude [deg, +ve = towards East]\
    #                 altitude [m]"
    # write(gps_data, gps_header, gps_filename)

    # gps_utm_filename = args.output_dir + "/gps-utm18t.txt"
    # gps_utm_data = bag_obj.load_gps_data(rel_time=False, ret_utm=True)
    # gps_utm_header = "time [s],\
    #                     UTM easting [m, +ve = towards East]\
    #                     UTM northing [m, +ve = towards North]\
    #                     altitude [m]"
    # write(gps_utm_data, gps_utm_header, gps_utm_filename)

    sun_ori_relative_filename = args.output_dir + "/relative-sun-position.txt"
    sun_ori_relative_data = bag_obj.load_sun_position_data(rel_time=False, reference="relative") 
    sun_ori_relative_header = "time [s],\
                                x_linear_position [m],\
                                y_linear_position [m],\
                                z_linear_position [m],\
                                x_orientation,\
                                y_orientation,\
                                z_orientation,\
                                w_orientation"
    write(sun_ori_relative_data, sun_ori_relative_header, sun_ori_relative_filename)

    sun_ori_global_filename = args.output_dir + "/global-sun-position.txt"
    sun_ori_global_data = bag_obj.load_sun_position_data(rel_time=False, reference="global") 
    sun_ori_global_header = "time [s],\
                                x_linear_position [m],\
                                y_linear_position [m],\
                                z_linear_position [m],\
                                x_orientation,\
                                y_orientation,\
                                z_orientation,\
                                w_orientation"
    write(sun_ori_global_data, sun_ori_global_header, sun_ori_global_filename)

if __name__ == "__main__":
    main()