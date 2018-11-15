#ifndef LASER_ONE_H
#define LASER_ONE_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <math.h>

#include "laser_base.h"

class LaserOne: public LaserBase
{
public:
	LaserOne();
	~LaserOne();

        /**
         * @brief LoadParams
         *        LoadParams loads parameters from the ROS parameter server
         *        Currently supported parameters:
         *        - laser_one/range_min: Fixes the minimum range of the sensor. Minimum range of the sensor is 0.5m
         *          Default value: 0.2
         *        - laser_one/range_max: Fixes the maximum range of the sensor. Maximum range of the sensor is 20m
         *          Default value: 20.0
         *        - laser_one/angle_min: Fixes the minimum FOV angle of the sensor. Minimum FOV angle of the sensor is -PI rad.
         *          Default value: -PI
         *        - laser_one/angle_max: Fixes the maximum FOV angle of the sensor. Maximum FOV angle of the sensor is PI rad.
         *          Default value: PI
         *        - laser_one/samples:   Fixes the number of data points/samples of the sensor. Maximum data points/samples of the sensor is 720.
         *          Default value: 720
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
	ros::Publisher laser_pub;
};

#endif // LASER_ONE_H
