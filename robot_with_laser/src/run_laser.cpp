#include <ros/ros.h>

#include "laser_one.h"
#include "laser_two.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "run_laser" );

    // "laser_name"
    //   - Description: Name of the desired laser
    //   - Default: LaserOne
    std::string laser_name;
    ros::param::param<std::string>("laser_name",laser_name, "LaserOne" );

    // "laser_name"
    //   - Description: Laser working frequency
    //   - Default: 30 (Hz)
    double laser_node_rate;
    ros::param::param<double>("laser_node_rate",laser_node_rate, 30.0 );

    //Saturate node rate if trying to run at a higher rate
    if(laser_node_rate > 30.0)
    		laser_node_rate = 30.0;

    LaserBase* laser;

    //Setup the desired laser.
    if(laser_name == "LaserOne")
    {
    	laser = new LaserOne();
    }
    else if(laser_name == "LaserTwo")
    {
    	laser = new LaserTwo();
    }
    else
    {
    	ROS_ERROR("Invalid Laser Name");
    	return 0;
    }

    //Load the required parameters
    laser->LoadParams();
    
    //Set if the values are incorrect
    laser->setRange();
    laser->setAngle();
    laser->setSamples();

    ros::Rate rate( laser_node_rate );

    while(ros::ok())
    {
        //Publish the scan message
    	laser->publishLaserScan();
    	rate.sleep();
    }

    delete laser;

    return 0;
}