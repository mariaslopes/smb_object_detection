#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_listener.h>
#include <object_detection_msgs/ObjectDetectionInfoArray.h>

void locationsCallback(const object_detection_msgs::ObjectDetectionInfoArray::ConstPtr& msg)
{   
    tf::Vector3 p_camera, p_world;
    std::string source_frame = msg->header.frame_id;
    tf::TransformListener listener;
    std::string base_frame = "odom";

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
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "object_process_node");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/object_detector/detection_info", 1, locationsCallback);

    ros::spin();

    return 0;
}
