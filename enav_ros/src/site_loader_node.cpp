/** site_loader_node.cpp

    Load the CSA site maps and publish them as a single grid_map ROS message

    Author: Olivier Lamarre <olivier.lamarre@robotics.utias.utoronto.ca>
    Affl:   Space and Terrestrial Autonomous Robotic Systems (STARS) Laboratory,
            University of Toronto
    Date:   July 17, 2019

**/


#include <ros/ros.h>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <opencv2/opencv.hpp>
#include <string>

#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>


// For debug purposes only
std::string GetImageType(int number)
{
    // find type
    int img_type_int = number%8;
    std::string img_type_string;

    switch (img_type_int)
    {
        case 0:
            img_type_string = "8U";
            break;
        case 1:
            img_type_string = "8S";
            break;
        case 2:
            img_type_string = "16U";
            break;
        case 3:
            img_type_string = "16S";
            break;
        case 4:
            img_type_string = "32S";
            break;
        case 5:
            img_type_string = "32F";
            break;
        case 6:
            img_type_string = "64F";
            break;
        default:
            break;
    }

    // find channel
    int channel = (number/8) + 1;

    std::stringstream type;
    type << "CV_" << img_type_string << "C" << channel;

    return type.str();
}


int main(int argc, char** argv)
{
    // Node initialization
    ros::init(argc, argv, "site_loader");
    ros::NodeHandle n;
    ros::Publisher publisher = n.advertise<grid_map_msgs::GridMap>("site_grid_map", 1, true);

    std::string dir_path = std::string(argv[1]);

    // Load geotiff rasters
    cv::Mat mosaic_rgb = cv::imread(dir_path + "/mosaic_utm_20cm.tif", cv::IMREAD_LOAD_GDAL | cv::IMREAD_COLOR );
    cv::Mat dem = cv::imread(dir_path + "/dem_utm_20cm.tif", cv::IMREAD_LOAD_GDAL | cv::IMREAD_ANYDEPTH);
    cv::Mat slope = cv::imread(dir_path + "/slope_utm_20cm.tif", cv::IMREAD_LOAD_GDAL | cv::IMREAD_ANYDEPTH);
    cv::Mat aspect = cv::imread(dir_path + "/aspect_utm_20cm.tif", cv::IMREAD_LOAD_GDAL | cv::IMREAD_ANYDEPTH);

    // Check if files were loaded properly
    if (mosaic_rgb.empty() || dem.empty() || slope.empty() || aspect.empty()) {
        ROS_ERROR("Could not load one or multiple maps from the following directory: %s", dir_path.c_str());

        std::string list_of_files = "mosaic_utm_20cm.tif, dem_utm_20cm.tif, slope_utm_20cm.tif, aspect_utm_20cm.tif";
        ROS_ERROR("Make sure the directory speficied contains the following files: %s", list_of_files.c_str());
        return -1; 
    }

    cv::cvtColor(mosaic_rgb, mosaic_rgb, cv::COLOR_BGR2RGB);
    
    // Rotate images 90 deg CCW (for alignment of corresponding grid maps)
    cv::Mat rot_mosaic_rgb(mosaic_rgb.cols, mosaic_rgb.rows, mosaic_rgb.type());
    cv::transpose(mosaic_rgb, rot_mosaic_rgb);
    cv::flip(rot_mosaic_rgb, rot_mosaic_rgb, 0);
    
    cv::Mat rot_dem(dem.cols, dem.rows, dem.type());
    cv::transpose(dem, rot_dem);
    cv::flip(rot_dem, rot_dem, 0);

    cv::Mat rot_slope(slope.cols, slope.rows, slope.type());
    cv::transpose(slope, rot_slope);
    cv::flip(rot_slope, rot_slope, 0);

    cv::Mat rot_aspect(aspect.cols, aspect.rows, aspect.type());
    cv::transpose(aspect, rot_aspect);
    cv::flip(rot_aspect, rot_aspect, 0);

    // Debug -- show image type
    // std::string ty =  GetImageType( mosaic_rgb.type() );
    // printf("Matrix: %s %dx%d \n", ty.c_str(), mosaic_rgb.cols, mosaic_rgb.rows);

    // Debug -- show Mat min and max
    // double min, max;
    // cv::minMaxLoc(mosaic_rgb, &min, &max);
    // printf("Min: %f \nMax: %f\n", min, max);

    // Convert opencv to Eigen
    Eigen::MatrixXf dem_eigen, slope_eigen, aspect_eigen;
    cv::cv2eigen(rot_dem, dem_eigen);
    cv::cv2eigen(rot_slope, slope_eigen);
    cv::cv2eigen(rot_aspect, aspect_eigen);

    // Set negative slope and aspects to 0
    slope_eigen = (slope_eigen.array() < 0).select(0, slope_eigen);
    aspect_eigen = (aspect_eigen.array() < 0).select(0, aspect_eigen);
    
    // Debug -- show loaded map size
    // printf("Cols: %d \nRows: %d\n", dem_eigen.cols(), dem_eigen.rows());

    // Create Grid Map
    grid_map::GridMap map;
    map.setFrameId("csa_origin");
    map.setGeometry(grid_map::Length(127.2, 68.6), 0.2, grid_map::Position(61.2,-30.0));

    // Add layers
    map.add("elevation", dem_eigen);
    map.add("slope", slope_eigen);
    map.add("aspect", aspect_eigen);

    map.add("color");
    for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
        const auto& cvColor = rot_mosaic_rgb.at<cv::Vec<unsigned char, 3>>((*iterator)(0), (*iterator)(1));
        Eigen::Vector3i colorVector;
        colorVector(0) = cvColor[0];
        colorVector(1) = cvColor[1];
        colorVector(2) = cvColor[2];
        grid_map::colorVectorToValue(colorVector, map.at("color", *iterator));
    }

    // Convert to ROS msg and publish
    map.setTimestamp(ros::Time::now().toNSec());
    grid_map_msgs::GridMap map_msg;
    grid_map::GridMapRosConverter::toMessage(map, map_msg);
    publisher.publish(map_msg);

    ros::Rate loop_rate(0.1);
    while (ros::ok())
    {
        ROS_INFO("Terrain maps published, spinning...");
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}