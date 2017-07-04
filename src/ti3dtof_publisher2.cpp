/*
 * ti3dtof_sensor component.
 *
 * Copyright (c) 2017 Texas Instruments Inc.
 */
#ifndef TI3DTOF_SENSOR_H
#define TI3DTOF_SENSOR_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include "CameraSystem.h"
#include "grabber.h"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "point_cloud_publisher");

    Voxel::CameraSystem _sys;
    std::vector< DevicePtr > _device = _sys.scan();
    std::map< String, Grabber* > _connected;
    int w = 0, h = 0;

    // Find all attached cameras 
    if (_device.size() <= 0) 
    {
        ROS_ERROR("No camera connected.");
        return -1;
    }

    DepthCameraPtr dc = _sys.connect(_device[0]);
    Grabber *grabber = new Grabber(dc, Grabber::FRAMEFLAG_ALL, _sys);
    grabber->getFrameSize(w, h);
    grabber->start();

    ros::NodeHandle n;
    ros::Publisher cloud_pub = n.advertise<sensor_msgs::PointCloud2>("cloud", 50);
    int count = 0;
    ros::Rate r(30.0);

    while(n.ok())
    {
        sensor_msgs::PointCloud2 cloud;
        cloud.header.stamp = ros::Time::now();
        cloud.header.frame_id = "sensor_frame";

        cloud.height = h;
        cloud.width = w;
        cloud.is_bigendian = true;
        cloud.fields.name = string("XYZI");
        cloud.offset = 0;
        cloud.datatype = sensor_msgs::FLOAT32;
        cloud.count = 4;


        if (grabber->getFrameCount() > 0) 
        {
            XYZIPointCloudFrame *frame = grabber->getXYZIFrame();
            if (frame) 
            {
                uint num_points = frame->points.size();
                cloud.points.resize(num_points);

                //we'll also add an intensity channel to the cloud
                cloud.channels.resize(1);
                cloud.channels[0].name = "intensities";
                cloud.channels[0].values.resize(num_points);

                //generate some fake data for our point cloud
                for (unsigned int i = 0; i < num_points; ++i)
                {
                    cloud.points[i].x = frame->points[i].x;
                    cloud.points[i].y = frame->points[i].y;
                    cloud.points[i].z = frame->points[i].z;
                    cloud.channels[0].values[i] = frame->points[i].i;
                }

                cloud_pub.publish(cloud);
                r.sleep();
                ++count;

                delete frame;

            } // if (frame)
        }
    }  
}

#endif // TI3DTOF_SENSOR 
/*! @} */

