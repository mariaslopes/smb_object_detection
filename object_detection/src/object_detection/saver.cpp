#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_listener.h>
#include <fstream>
#include <object_detection_msgs/ObjectDetectionInfoArray.h>

std::string filename, base_frame;
std::ofstream outFile;

void locationsCallback(const object_detection_msgs::ObjectDetectionInfoArray::ConstPtr& msg)
{   
    tf::Vector3 p_camera, p_world;
    std::string source_frame = msg->header.frame_id;
    tf::TransformListener listener;
    std::string base_frame = "world_graph_msf";

    for (int i = 0; i < msg->info.size(); i++)
    {   
        std::string object_id = msg->info.at(i).class_id;
        p_camera.setX(msg->info.at(i).position.x);
        p_camera.setY(msg->info.at(i).position.y);
        p_camera.setZ(msg->info.at(i).position.z);

        tf::StampedTransform transform;
        try{
            listener.waitForTransform(base_frame, source_frame, ros::Time(0), ros::Duration(1.0));
            listener.lookupTransform(base_frame, source_frame, ros::Time(0), transform);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
        }

        p_world = transform*p_camera;
        
        ROS_INFO("Identified object %s at position %f %f %f", object_id.c_str(), p_world.getX(), p_world.getY(), p_world.getZ());
        outFile << object_id << "," << p_world.getX() << "," << p_world.getY() << "," << p_world.getX() << "," << msg->info.at(i).confidence <<"\n";
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "object_process_node");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/object_detector/detection_info", 1, locationsCallback);

    if (!nh.getParam("/object_processor/base_frame", base_frame)) return 0;
    std::cout << "Read frame";
    if (!nh.getParam("/object_processor/csv_path", filename)) return 0;
    std::cout << "Read path";
    
    outFile.open(filename);
    if (!outFile.is_open()) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return 1;
    }
    outFile << "Object,X,Y,Z,Confidence\n";

    ros::spin();

    return 0;
}
