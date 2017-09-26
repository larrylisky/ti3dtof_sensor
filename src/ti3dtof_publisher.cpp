/*
 * ti3dtof_sensor component.
 *
 * Copyright (c) 2017 Texas Instruments Inc.
 */
#ifndef TI3DTOF_SENSOR_H
#define TI3DTOF_SENSOR_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include "CameraSystem.h"
#include "grabber.h"
#include <limits>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "ti3dtof_sensor_publisher");

    Voxel::CameraSystem _sys;
    std::vector< DevicePtr > _device = _sys.scan();
    std::map< String, Grabber* > _connected;
    int width, height;

    // Find all attached cameras 
    if (_device.size() <= 0) 
    {
        ROS_ERROR("No camera connected.");
        return -1;
    }

    DepthCameraPtr dc = _sys.connect(_device[0]);
    
    // Setup ROS
    ros::NodeHandle n;
    ros::Publisher laser_pub, cloud_pub;

    std::string laserTopic, pclTopic;
    float hfov, vfov;
    float frameRate;
    int hStart, hEnd;
    float minRange, maxRange;
    float flypixThr;
    float gain;
    int order;

    ros::NodeHandle priv_n("~");
    priv_n.param("laser", laserTopic, std::string("scan"));
    priv_n.param("pcl", pclTopic, std::string("pcl"));
    priv_n.param("hfov", hfov, 74.2f);
    priv_n.param("vfov", vfov, 58.1f);
    priv_n.param("rate", frameRate, 30.0f);
    priv_n.param("hStart", hStart, 0);
    priv_n.param("hEnd", hEnd, 120);
    priv_n.param("minRange", minRange, 0.0f);
    priv_n.param("maxRange", maxRange, 5.0f);
    priv_n.param("flypixThr", flypixThr, 500.0f);
    priv_n.param("order", order, 1);
    priv_n.param("gain", gain, 1.0f);
    priv_n.param("width", width, 160);
    priv_n.param("height", height, 120);

#if 1
    std::cout << "~laser := " << laserTopic << std::endl;
    std::cout << "~pcl := " << pclTopic << std::endl;
    std::cout << "~hfov := " << hfov << std::endl;
    std::cout << "~vfov := " << vfov << std::endl;
    std::cout << "~rate := " << frameRate << std::endl;
    std::cout << "~hStart := " << hStart << std::endl;
    std::cout << "~hEnd := " << hEnd << std::endl;
    std::cout << "~minRange := " << minRange << std::endl;
    std::cout << "~maxRange := " << maxRange << std::endl;
    std::cout << "~flypixThr := " << flypixThr << std::endl;
    std::cout << "~order := " << order << std::endl;
    std::cout << "~gain := " << gain << std::endl;
    std::cout << "~width = " << width << std::endl;
    std::cout << "~height = " << height << std::endl;
#endif

    // Setup grabber dimension
    FrameSize dim;
    dim.width = width;
    dim.height = height;
    Grabber *grabber = new Grabber(dc, dim, Grabber::FRAMEFLAG_ALL, _sys);


    // Setup ToF filters
    FilterPtr p;
#if 0
    p = grabber->createFilter("Voxel::TemporalMedianFilter",
		DepthCamera::FRAME_RAW_FRAME_PROCESSED);
    if (!p) 
        logger(LOG_ERROR) << "Failed to get TemporalMedianFilter" << std::endl;
    p->set("deadband", 0.0f);
    p->set("order", (uint)order);
    grabber->addFilter(p, DepthCamera::FRAME_RAW_FRAME_PROCESSED);
#endif

    p = grabber->createFilter("Voxel::MedianFilter", DepthCamera::FRAME_RAW_FRAME_PROCESSED);
    if (!p) logger(LOG_ERROR) << "Failed to get MedianFilter" << std::endl;
	p->set("deadband", 0.0f);
    grabber->addFilter(p, DepthCamera::FRAME_RAW_FRAME_PROCESSED);

    p = grabber->createFilter("Voxel::FlypixFilter", DepthCamera::FRAME_RAW_FRAME_PROCESSED);
    if (!p) 
        logger(LOG_ERROR) << "Failed to get FlypixFilter" << std::endl;
    p->set("threshold", flypixThr);
    grabber->addFilter(p, DepthCamera::FRAME_RAW_FRAME_PROCESSED);

    // Start ToF camera
    grabber->start();

    // Advertize ROS topics
    if (laserTopic.c_str())
        laser_pub = n.advertise<sensor_msgs::LaserScan>(laserTopic.c_str(), 1);
    if (pclTopic.c_str())
        cloud_pub = n.advertise<sensor_msgs::PointCloud>(pclTopic.c_str(), 1);

    int count = 0;
    ros::Rate r(frameRate);

    sensor_msgs::LaserScan laser;
    laser.header.frame_id = "ti3dtof_frame";
    laser.angle_min = -hfov*M_PI/180.0/2.0;
    laser.angle_max = hfov*M_PI/180.0/2.0;
    laser.angle_increment = (laser.angle_max - laser.angle_min)/width;
    laser.scan_time = 1.0/frameRate;
    laser.range_min = minRange;
    laser.range_max = maxRange;     
    laser.ranges.resize(width);
    laser.intensities.resize(width);

    sensor_msgs::PointCloud cloud;
    cloud.header.frame_id = laser.header.frame_id;

    while(n.ok())
    {
        laser.header.stamp = ros::Time::now();
        cloud.header.stamp = laser.header.stamp;

        if (grabber->getFrameCount() > 0) 
        {
            XYZIPointCloudFrame *pclFrame = grabber->getXYZIFrame();
    
            // Laser scan

            if (laserTopic.c_str())
            {
                if (pclFrame)
                {
                    for (int j = 0; j < width; j++)
                    {
                        float minDist = std::numeric_limits<float>::max();
                        float intensity = std::numeric_limits<float>::max();

                        for (int i = hStart; i < hEnd; i++)
                        {
                            int idx = i*width+j;
                            float px = pclFrame->points[idx].x;
                            float py = pclFrame->points[idx].y;
                            float pz = pclFrame->points[idx].z;
                            float range = sqrt(px*px+py*py+pz*pz); 
                            float amplitude = pclFrame->points[idx].i;
                            
                            if (range > 0 && range < minDist)
                            {
                                minDist = range;
                                intensity = amplitude;
                            }
                        }
                        laser.ranges[width-1-j] = minDist;
                        laser.intensities[width-1-j] = gain * intensity;
                    }

                    laser_pub.publish(laser);

                } // if (pclFrame)
            }


            // Point Cloud 
            if (pclTopic.c_str())
            {
                if (pclFrame) 
                {
                    int num_points = pclFrame->points.size();
                    cloud.points.resize(num_points);

                    //we'll also add an intensity channel to the cloud
                    cloud.channels.resize(1);
                    cloud.channels[0].name = "intensities";
                    cloud.channels[0].values.resize(num_points);

                    //generate some fake data for our point cloud
                    for (int i = 0; i < num_points; ++i)
                    {
                        cloud.points[i].x = pclFrame->points[i].z;
                        cloud.points[i].y = -pclFrame->points[i].x;
                        cloud.points[i].z = -pclFrame->points[i].y;
                        cloud.channels[0].values[i] = gain * pclFrame->points[i].i;
                        if (abs(cloud.points[i].x) > maxRange) 
                        {
                            cloud.points[i].x = 0; 
                            cloud.points[i].y = 0;
                            cloud.points[i].z = 0;
                            cloud.channels[0].values[i] = 0;
                        } 
                    }

                    cloud_pub.publish(cloud);

                } // if (pclFrame)
            }
                
            if (pclFrame)    
                delete pclFrame;

            r.sleep();
            ++count;
        }
    }
  
}


#endif // TI3DTOF_SENSOR 
/*! @} */

