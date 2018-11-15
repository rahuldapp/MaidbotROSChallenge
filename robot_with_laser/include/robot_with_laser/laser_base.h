#ifndef LASER_BASE_H
#define LASER_BASE_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <math.h>

class LaserBase
{
public:
	LaserBase();
	virtual ~LaserBase();

        /**
         * @brief LoadParams
         *        LoadParams loads parameters from the ROS parameter server
         */
	virtual void LoadParams();

        /**
         * @brief publishLaserScan
         *        publishLaserScan publishes the fake laserscan messages for a sensor
         */
	virtual void publishLaserScan();

        /**
         * @brief setRange
         *        setRange sets the minimum and the maximum range of the sensor if parameters are inappropriate
         */
	virtual void setRange();

        /**
         * @brief setAngle
         *        setAngle sets the minimum and the maximum FOV angle of the sensor if parameters are inappropriate
         */
	virtual void setAngle();

        /**
         * @brief setSamples
         *        setSamples sets the number of data points/samples of the sensor if parameters are inappropriate
         */
	virtual void setSamples();

protected:

	ros::NodeHandle node_handle;

	sensor_msgs::LaserScan scan_msg;
	double angle_min, angle_max;
	double range_min, range_max;
	int num_samples;
	
};


#endif // LASER_BASE_H
