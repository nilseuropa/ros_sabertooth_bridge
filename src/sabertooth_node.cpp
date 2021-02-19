#include <ros/ros.h>
#include <ros/console.h>
#include <serial/serial.h>
#include <string>
#include <sstream>
#include <ctime>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>

ros::Timer        twist_command_watchdog;
ros::Timer        left_motor_cmd_watchdog;
ros::Timer        right_motor_cmd_watchdog;

serial::Serial    controller;

void left_wheel_power_cmd_cb(const std_msgs::Float32& left_power){
    left_motor_cmd_watchdog.stop();
    // send_motor_power
    left_motor_cmd_watchdog.start();
}
void left_motor_watchdog_cb(const ros::TimerEvent&){
  left_motor_cmd_watchdog.stop();
  // stop_motor
}

void right_wheel_power_cmd_cb(const std_msgs::Float32& right_power){
    right_motor_cmd_watchdog.stop();
    // send_motor_power
    right_motor_cmd_watchdog.start();
}
void right_motor_watchdog_cb(const ros::TimerEvent&){
  right_motor_cmd_watchdog.stop();
  // stop_motor
}

double wheel_separation = 0;
void cmd_vel_cb(const geometry_msgs::Twist& twist)
{
  twist_command_watchdog.stop();
  float left_track_linear_velocity  = (2.0f * twist.linear.x - wheel_separation * twist.angular.z)/2.0f;
  float right_track_linear_velocity = (2.0f * twist.linear.x + wheel_separation * twist.angular.z)/2.0f;
  // send_motor_power(s)
  twist_command_watchdog.start();
}
void twist_watchdog_cb(const ros::TimerEvent&)
{
  twist_command_watchdog.stop();
  // stop_motors
  controller.flush();
}


int main(int argc, char **argv){

    ros::init(argc, argv, "sabertooth_node");
    ros::NodeHandle n;
    ros::NodeHandle nhLocal("~");

    double refresh_rate;
    nhLocal.param("refresh_rate", refresh_rate, 1000.0);

    std::string port;
    nhLocal.param<std::string>("port", port, "/dev/ttyACM0");

    int baud;
    nhLocal.param("baud", baud, 115200);

    int watchdog_timeout;
    nhLocal.param("watchdog_timeout", watchdog_timeout, 100);

    bool enable_twist_input;
    nhLocal.param("enable_twist_input",enable_twist_input, false);
    nhLocal.param("wheel_separation", wheel_separation, 0.56);

    // set up watchdogs
    serial::Timeout timeout = serial::Timeout::simpleTimeout(watchdog_timeout);
    twist_command_watchdog = n.createTimer(ros::Duration(watchdog_timeout/1000.0), twist_watchdog_cb, true);
    left_motor_cmd_watchdog = n.createTimer(ros::Duration(watchdog_timeout/1000.0), left_motor_watchdog_cb, true);
    right_motor_cmd_watchdog = n.createTimer(ros::Duration(watchdog_timeout/1000.0), right_motor_watchdog_cb, true);

    // set up serial port
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

    ros::Subscriber right_wheel_cmd = n.subscribe("/right_motor/voltage_norm", 10, right_wheel_power_cmd_cb);
    ros::Subscriber left_wheel_cmd  = n.subscribe("/left_motor/voltage_norm", 10, left_wheel_power_cmd_cb);

    ros::Subscriber twist_cmd_sub;
    if (enable_twist_input) {
        twist_cmd_sub = n.subscribe("/cmd_vel", 10, cmd_vel_cb);
        ROS_INFO("Twist input enabled.");
    }

    ros::Rate rate(refresh_rate);
    ROS_INFO("%s started.", ros::this_node::getName().c_str());

    while (ros::ok())
    {
        ros::spinOnce();
        // read_stream();
        rate.sleep();
    }

    // Shutdown
    if (controller.isOpen()) controller.close();
    ROS_INFO("Drive node exiting...");
    return 0;
}
