#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>


void cloud_callback (const sensor_msgs::PointCloud& cloud){
//     ROS_INFO("inside callback");
    int num_points = cloud.points.size();
    
    for(unsigned int i = 0; i < num_points; ++i){
       std::cout << "(" 
                 << cloud.points[i].x 
                 << ", " 
                 << cloud.points[i].y 
                 << ", " 
      		 << cloud.points[i].z
                 << " )" 
                 << std::endl; 
    }

}

int main (int argc, char** argv) {
     ros::init (argc, argv, "cloud_sub");
     ros::NodeHandle n;
     ros::Rate loop_rate(10);
     ros::Subscriber sub;
     sub = n.subscribe ("cloud", 1, cloud_callback);
     ros::spin();
 }
