#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <control_toolbox/pid.h>
#include <vector>

// Initialisiere PIDs
std::vector<control_toolbox::Pid> pids;

// Sollwerte der Gelenke
std::vector<double> target_joint_angles(7, 1.0);  

// Aktuelle Gelenkpositionen
std::vector<double> current_joint_angles(7, 0.0); 

// Kontrollsignale der PIDs
std_msgs::Float64MultiArray control_signals;

// Rückruffunktion für Gelenkzustände
void joint_states_callback(const sensor_msgs::JointState::ConstPtr& msg)
{
    current_joint_angles = msg->position;
}

// Rückruffunktion für Zielgelenkpositionen
void desired_positions_callback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    target_joint_angles = msg->data;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "control_pid_node");
    ros::NodeHandle nh;

    pids.resize(7);

    for (int i = 0; i < 7; i++) {
        // Verwende den vollständigen Namespace
        if (!pids[i].init(ros::NodeHandle(nh, "gazebo_ros_control/pid_gains/panda_joint" + std::to_string(i+1)))) {
            ROS_FATAL("Failed to initialize PID for panda_joint%d", i+1);
            return 1;
        }
    }

    ros::Subscriber sub_joint_states = nh.subscribe("/joint_states", 100, joint_states_callback);
    ros::Subscriber sub_desired_positions = nh.subscribe("/desired_joint_positions", 100, desired_positions_callback);

    ros::Publisher pub = nh.advertise<std_msgs::Float64MultiArray>("/joint_position_example_controller_sim/joint_command", 100);

    ros::Rate loop_rate(100);  // 100 Hz

    while (ros::ok())
    {
        // Führe PID-Regelung durch und speichere Ergebnisse in control_signals
        control_signals.data.clear();
        for (int i = 0; i < 7; i++) {
            double error = target_joint_angles[i] - current_joint_angles[i];
            control_signals.data.push_back(pids[i].computeCommand(error, ros::Duration(loop_rate.expectedCycleTime())));
        }

        // Veröffentliche Steuersignale
        pub.publish(control_signals);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
