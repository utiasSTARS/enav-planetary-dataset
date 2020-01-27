/** omni_tf_broadcaster.cpp

    TF broadcaster for each camera on the omnidirectional stereovisual sensor

    Author: Olivier Lamarre <olivier.lamarre@robotics.utias.utoronto.ca>
    Affl:   Space and Terrestrial Autonomous Robotic Systems (STARS) Laboratory,
            University of Toronto
    Date:   July 17, 2019

**/


#include <sstream>
#include <vector>

#include <ros/ros.h>
#include <tf2/LinearMath/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>


struct OccamCameraTransformROS {

    int sensor_no, ros_no;
    
    tf2::Quaternion q;
    tf2::Vector3 T;

    std::string ros_no_str(){
        // Return ros_no as a string

        std::stringstream ss;
        ss << ros_no;

        return ss.str();
    }
};


std::vector<OccamCameraTransformROS> LoadOccamTransforms(ros::NodeHandle& nh) {

    std::vector<OccamCameraTransformROS> occam_transforms;
    occam_transforms.resize(10);
    
    std::string occam_sensor_id[] = {"/omni0", "/omni1", "/omni2", "/omni3", "/omni4", "/omni5", "/omni6", "/omni7", "/omni8", "/omni9"};
    int occam_ros_id[] = {0,2,4,6,8,1,3,5,7,9};

    for ( int i = 0; i < occam_transforms.size(); i++ ) {
        
        // Initialize current OccamCameraTransformROS instance
        OccamCameraTransformROS oct;
        oct.sensor_no = i;
        oct.ros_no = occam_ros_id[i];
        
        // Check for the existence of the requested parameter
        std::string rotation_param_name = occam_sensor_id[i]+"/R/data";
        std::string translation_param_name = occam_sensor_id[i]+"/T/data";

        if (!nh.hasParam(rotation_param_name) || !nh.hasParam(translation_param_name)) {
            ROS_ERROR_STREAM("Rotation parameter '" << rotation_param_name << "' or translation parameter '"
                      << translation_param_name << "' not found!");
        }

        // Get rotation & translation
        std::vector<double> R, T;
        nh.getParam(rotation_param_name, R);
        nh.getParam(translation_param_name, T);

        // Store them into a transform
        tf2::Matrix3x3 R_matrix = tf2::Matrix3x3(tf2Scalar(R[0]), tf2Scalar(R[1]), tf2Scalar(R[2]), tf2Scalar(R[3]), tf2Scalar(R[4]),
                                                 tf2Scalar(R[5]), tf2Scalar(R[6]), tf2Scalar(R[7]), tf2Scalar(R[8]));
        tf2::Vector3 T_origin = tf2::Vector3(tf2Scalar(T[0]), tf2Scalar(T[1]), tf2Scalar(T[2]));
        tf2::Transform transform = tf2::Transform(R_matrix, T_origin);
        
        // OCCAM gives the inverse of the transform expected by ROS
        tf2::Transform transform_inv = transform.inverse();

        oct.T = transform_inv.getOrigin();
        oct.q = transform_inv.getRotation();

        occam_transforms[i] = oct;
    }
    
    return occam_transforms;
}


void BroadcastAllTransforms(std::vector<OccamCameraTransformROS>& occam_transforms){

    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transform_stamped;
    ros::Time stamp = ros::Time::now();

    for ( int i = 0; i < occam_transforms.size(); i++ ) {
        
        transform_stamped.header.stamp = stamp;
        transform_stamped.header.frame_id = "omni_ref_link";
        transform_stamped.child_frame_id = "omni"+occam_transforms[i].ros_no_str();

        transform_stamped.transform.translation.x = occam_transforms[i].T[0]/1000;
        transform_stamped.transform.translation.y = occam_transforms[i].T[1]/1000;
        transform_stamped.transform.translation.z = occam_transforms[i].T[2]/1000;
        
        transform_stamped.transform.rotation.x = occam_transforms[i].q.x();
        transform_stamped.transform.rotation.y = occam_transforms[i].q.y();
        transform_stamped.transform.rotation.z = occam_transforms[i].q.z();
        transform_stamped.transform.rotation.w = occam_transforms[i].q.w();

        br.sendTransform(transform_stamped);
    }
    
    return;
}


int main(int argc, char** argv){
    
    ros::init(argc, argv, "omni_tf_broadcaster");
    ros::NodeHandle nh_private("~");

    // Load all OCCAM transforms
    ROS_INFO("Loading omnidirectional camera transforms ...");
    std::vector<OccamCameraTransformROS> occam_transforms = LoadOccamTransforms(nh_private);

    // Broadcast loaded transforms
    ros::Rate r(10);
    while (ros::ok()){
        BroadcastAllTransforms(occam_transforms);
        r.sleep();
    }

    return 0;
};