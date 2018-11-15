#ifndef LASER_TWO_H
#define LASER_TWO_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <math.h>
#include "laser_base.h"

class LaserTwo: public LaserBase
{
public:
	LaserTwo();
	~LaserTwo();

	/**
     * @brief LoadParams
     *        LoadParams loads parameters from the ROS parameter server
     *        Currently supported parameters:
     *        - laser_two/range_min: Fixes the minimum range of the sensor. Minimum range of the sensor is 0.2m
     *          Default value: 0.2
     *        - laser_two/range_max: Fixes the maximum range of the sensor. Maximum range of the sensor is 10m
     *          Default value: 20.0
     *        - laser_two/angle_min: Fixes the minimum FOV angle of the sensor. Minimum FOV angle of the sensor is -PI rad.
     *          Default value: -PI
     *        - laser_two/angle_max: Fixes the maximum FOV angle of the sensor. Maximum FOV angle of the sensor is PI rad.
     *          Default value: PI
     *        - laser_two/samples:   Fixes the number of data points/samples of the sensor. Maximum data points/samples of the sensor is 1440.
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


#endif // LASER_TWO_H