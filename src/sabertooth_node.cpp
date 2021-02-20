/*

Based on:
Sabertooth Simplified Serial Protocol (2012-2013 Dimension Engineering LLC)

*/

#include <ros/ros.h>
#include <ros/console.h>
#include <serial/serial.h>
#include <string>
#include <sstream>
#include <ctime>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>

#define MOTOR_MAX 2047

serial::Serial    controller;

double map(double x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
};

void set_motor_power(uint8_t motor, int power){
  std::stringstream cmd;
  cmd << "M" << (char)motor+0 <<": " << power << "\r\n";
  std::cout << cmd.str();
  controller.write(cmd.str());
}

void motor_cb(const std_msgs::Float32& power, uint8_t motor){
  float p = map(power.data, -1.0, 1.0, -MOTOR_MAX, MOTOR_MAX);
  set_motor_power(motor,(int)p);
}

int main(int argc, char **argv){

    ros::init(argc, argv, "sabertooth_node");
    ros::NodeHandle n;
    ros::NodeHandle nhLocal("~");

    int num_of_channels;
    nhLocal.param("num_of_channels", num_of_channels, 4);

    // set up watchdogs
    int watchdog_timeout;
    nhLocal.param("watchdog_timeout", watchdog_timeout, 100);
    serial::Timeout timeout = serial::Timeout::simpleTimeout(watchdog_timeout);

    // set up serial port
    std::string port;
    nhLocal.param<std::string>("port", port, "/dev/ttyACM0");
    int baud;
    nhLocal.param("baud", baud, 115200);
    controller.setPort(port);
    controller.setBaudrate(baud);
    controller.setTimeout(timeout);

    while ( ros::ok() ) {
        ROS_INFO_STREAM("Opening serial port on " << port << " at " << baud << "..." );
        try {
            controller.open();
            if ( controller.isOpen() )
            {
                ROS_INFO("Successfully opened serial port.");
                break;
            }
        } catch (serial::IOException e)
        {
            ROS_ERROR_STREAM("serial::IOException: " << e.what());
        }
        ROS_ERROR("Failed to open serial port");
        ROS_WARN("Retry in 5 seconds.");
        sleep( 5 );
    }


    std::vector<ros::Subscriber> subs;
    for (uint8_t index=1; index<=num_of_channels; index++){
      boost::function<void (const std_msgs::Float32&)> f = boost::bind(motor_cb, _1, index);
      std::stringstream topicName;
      topicName << "/motor_" << index+0 << "/power";
      subs.push_back(n.subscribe<std_msgs::Float32>(topicName.str(), 10, f ));
      ROS_INFO("Subscribed to topic %s", subs.back().getTopic().c_str());
    }

    ROS_INFO("%s started.", ros::this_node::getName().c_str());
    ros::spin();

    /*
    ros::Rate rate(refresh_rate);
    while (ros::ok())
    {
        ros::spinOnce();
        // read_stream();
        rate.sleep();
    }
    */

    // Shutdown
    if (controller.isOpen()) controller.close();
    ROS_INFO("Drive node exiting...");
    return 0;
}
