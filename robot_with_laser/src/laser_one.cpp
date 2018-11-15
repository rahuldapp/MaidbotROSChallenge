#include "laser_one.h"

LaserOne::LaserOne()
{
	laser_pub = node_handle.advertise<sensor_msgs::LaserScan>("scan", 1);
	return;
}

void LaserOne::LoadParams()
{
	// "laser_one/range_min"
    //   - Description: Minimum range of the sensor (m)
    //   - Default: 0.5
    ros::param::param<double>("laser_one/range_min",range_min, 0.5 );

    // "laser_one/range_max"
    //   - Description: Maximum range of the sensor (m)
    //   - Default: 20
    ros::param::param<double>("laser_one/range_max",range_max, 20.0 );

    // "laser_one/angle_min"
    //   - Description: Minimum FOV angle of the sensor (rad)
    //   - Default: -PI
    ros::param::param<double>("laser_one/angle_min",angle_min, -M_PI );

    // "laser_one/angle_max"
    //   - Description: Maximum FOV angle of the sensor (rad)
    //   - Default: PI
    ros::param::param<double>("laser_one/angle_max",angle_max, M_PI );

    // "laser_one/samples"
    //   - Description: Number of data points or samples of the sensor
    //   - Default: 720
    ros::param::param<int>("laser_one/samples",num_samples, 720);
}

void LaserOne::setRange()
{
	//saturate the minimum and maximum range values if values given are erroneous
	if(range_min > 20.0)
		range_min = 20.0;
	if(range_min < 0.5)
		range_min = 0.5;

	if(range_max > 20.0)
		range_max = 20.0;
	if(range_max < 0.5)
		range_max = 0.5;
}

void LaserOne::setAngle()
{
	//saturate the minimum and maximum FOV angle values if values given are erroneous
	if(angle_min > M_PI)
		angle_min = M_PI;
	if(angle_min < -M_PI)
		angle_min = -M_PI;

	if(angle_max > M_PI)
		angle_max = M_PI;
	if(angle_max < -M_PI)
		angle_max = -M_PI;
}

void LaserOne::setSamples()
{
	//saturate the number of samples if values given are erroneous
	if(num_samples > 720)
		num_samples = 720;
	if(num_samples < 0)
		num_samples = 0;
}

void LaserOne::publishLaserScan()
{
	//publish fake laserscan data
	scan_msg.header.stamp = ros::Time::now();
    scan_msg.header.frame_id = "laser";

    scan_msg.angle_min = angle_min;
    scan_msg.angle_max = angle_max;

    scan_msg.angle_increment = double((angle_max-angle_min)/num_samples);

    scan_msg.time_increment = 0;
    scan_msg.scan_time = 0;

    scan_msg.range_min = range_min;
    scan_msg.range_max = range_max;

    scan_msg.ranges.resize(num_samples);


    //laser_one publishes half points as min range and half points as max range - 0.01, since if range = max_range, the value would be discarded. 
    for(int i = 0 ; i < num_samples/2 ; i ++)
    {
        scan_msg.ranges[i] = range_min;
    }
    for(int i = num_samples/2 ; i < num_samples ; i ++)
    {
        scan_msg.ranges[i] = range_max - 0.01;
    }

	laser_pub.publish(scan_msg);

}

LaserOne::~LaserOne()
{
	return;
}