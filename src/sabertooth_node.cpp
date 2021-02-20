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

serial::Serial    controller;

#define MOTOR_MAX 2047

double map(double x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
};

void set_motor_power(uint8_t motor, int power){
  std::stringstream cmd;
  cmd << "M" << motor << ": " << power << "\r\n";
  controller.write(cmd.str());
}

void m1_cb(const std_msgs::Float32& power){
  float p = map(power.data, -1.0, 1.0, -MOTOR_MAX, MOTOR_MAX);
  set_motor_power(1,(int)p);
}

void m2_cb(const std_msgs::Float32& power){
  float p = map(power.data, -1.0, 1.0, -MOTOR_MAX, MOTOR_MAX);
  set_motor_power(2,(int)p);
}

void m3_cb(const std_msgs::Float32& power){
  float p = map(power.data, -1.0, 1.0, -MOTOR_MAX, MOTOR_MAX);
  set_motor_power(3,(int)p);
}

void m4_cb(const std_msgs::Float32& power){
  float p = map(power.data, -1.0, 1.0, -MOTOR_MAX, MOTOR_MAX);
  set_motor_power(4,(int)p);
}


int main(int argc, char **argv){

    ros::init(argc, argv, "sabertooth_node");
    ros::NodeHandle n;
    ros::NodeHandle nhLocal("~");

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

    ros::Subscriber m1 = n.subscribe("/m1/power", 10, m1_cb);
    ros::Subscriber m2 = n.subscribe("/m2/power", 10, m2_cb);
    ros::Subscriber m3 = n.subscribe("/m3/power", 10, m3_cb);
    ros::Subscriber m4 = n.subscribe("/m4/power", 10, m4_cb);

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
