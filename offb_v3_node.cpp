/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <math.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Int16.h>
#include "offb_v3/pid.hpp"
#include <offb_v3.msg/Targetmsg.h>


mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

geometry_msgs::PoseStamped current_position;
void callback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_position = *msg;
}
std_msgs::Int16 qr_tengah;
void callback2(const std_msgs::Int16::ConstPtr& msg){
    qr_tengah = *msg;
}
std_msgs::Int16 qr_kiri;
void callback3(const std_msgs::Int16::ConstPtr& msg){
    qr_kiri = *msg;
}
std_msgs::Int16 qr_kanan;
void callback4(const std_msgs::Int16::ConstPtr& msg){
    qr_kiri = *msg;
}
offb_v3::Targetmsg center_qr;
void callback5(const offb_v3::Targetmsg::ConstPtr& msg){
    center_qr = *msg;
}
float jarak_titik(float tujuan_x, float tujuan_y, float tujuan_z, float asal_x, float asal_y, float asal_z){
    float jarak_x = tujuan_x - asal_x;
    float jarak_y = tujuan_y - asal_y;
    float jarak_z = tujuan_z - asal_z;
    float jarak_total = sqrt(pow(jarak_x, 2) + pow(jarak_y, 2) + pow(jarak_z, 2));
    return jarak_total;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    double vel_x, vel_y, vel_z;
    bool takeoff = false;
    int qrcode;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist> //Publish kecepatan drone
            ("mavros/setpoint_velocity/cmd_vel_unstamped", 10);
    ros::Subscriber current_pos = nh.subscribe<geometry_msgs::PoseStamped> //Subscribe posisi drone
            ("mavros/local_position/pose", 10, callback);
    ros::Subscriber topic_qr_tengah = nh.subscribe<std_msgs::Int16> //Subscribe qrcode meja 1 (tengah)
            ("front/tengah", 10, callback2);
    ros::Subscriber topic_qr_kiri = nh.subscribe<std_msgs::Int16> //Subscribe qrcode 2 (kiri)
            ("front/kiri", 10, callback3);       
    ros::Subscriber topic_qr_kanan = nh.subscribe<std_msgs::Int16> //Subscribe qrcode 3 (kanan)
            ("front/kanan", 10, callback4);
    ros::Subscriber current_qr = nh.subscribe<offb_v3::Targetmsg>
            ("front/processed", 10, callback5)

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(32.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.z = 0;


    geometry_msgs::Twist cmd_msg;
    cmd_msg.linear.x = 0;
    cmd_msg.linear.y = 0;
    cmd_msg.linear.z = 0;
    cmd_msg.angular.x = 0;
    cmd_msg.angular.y = 0;
    cmd_msg.angular.z = 0;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        cmd_pub.publish(cmd_msg);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    int qop;
    std::cout << "Mulai? (y/n) ";
    std::cin >> qop;

    while(ros::ok() && (current_state.mode != "OFFBOARD" || !current_state.armed)){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        cmd_pub.publish(cmd_msg);
        ros::spinOnce();
        rate.sleep();
    }


    PID mypid_x(0.03125, 1, -1, 0.75, 0.01, 0.0001);
    PID mypid_y(0.03125, 1, -1, 0.75, 0.01, 0.0001);
    PID mypid_z(0.03125, 1, -1, 0.75, 0.01, 0.0001);

    ros::Time stay;

    float lokasi_meja[][3] = {
        {0, 0, 0},
        {8, 0, 2}, //tengah
        {8, 3.3, 2.5}, //kiri
        {8, -3.3, 3}, //kanan
    };
    float lokasi_misi[][3] = {
        {0, 0, 0},
        {},
        {},
        {},
    }
    int meja_tujuan = 1; 
    int misi_tujuan = 1;

    while(ros::ok() && current_state.mode == "OFFBOARD" && current_state.armed){
        if (meja_tujuan == 3){
            for (int i = 1; i <= 3; i++){
                if (lokasi_misi[i] == {}){
                    lokasi_misi[i] = lokasi_meja[3];
                }
            }
        }

        if (!takeoff){
            cmd_msg.linear.x = mypid_x.calculate(0, current_position.pose.position.x);
            cmd_msg.linear.y = mypid_y.calculate(0, current_position.pose.position.y);
            cmd_msg.linear.z = mypid_z.calculate(2, current_position.pose.position.z);
            cmd_pub.publish(cmd_msg);

            if (current_position.pose.position.z < 1.9){
                stay = ros::Time::now();
                ROS_INFO("Taking off!");
            }
            else if (ros::Time::now() - stay > ros::Duration(1.5)){
                takeoff = true;
                ROS_INFO("Sudah takeoff!");
            }
        }
        else if (lokasi_misi[misi_tujuan] == {} && misi_tujuan < 4){
            cmd_msg.linear.x = mypid_x.calculate(lokasi_meja[meja_tujuan][0], current_position.pose.position.x);
            cmd_msg.linear.y = mypid_y.calculate(lokasi_meja[meja_tujuan][1], current_position.pose.position.y);
            cmd_msg.linear.z = mypid_z.calculate(loksai_meja[meja_tujuan][2], current_position.pose.position.z);
            cmd_pub.publish(cmd_msg);
            //subs data qrcode
            if (meja_tujuan == 1){
                qrcode = data_tengah.data; 
            }
            else if (meja_tujuan == 2){
                qrcode = data_kiri.data; 
            }
            else if (meja_tujuan == 3){
                qrcode = data_kanan.data;
            }

            if (qrcode){
                if (qrcode == misi_tujuan){  
                    lokasi_misi[misi_tujuan] = lokasi_meja[meja_tujuan];   
                }
                else if (qrcode != misi_tujuan){
                    lokasi_misi[qrcode] = lokasi_meja[meja_tujuan];
                    meja_tujuan += 1;
                }
            }
            

        }
        else if (lokasi_misi[misi_tujuan] != {} && misi_tujuan < 4){
            if (jarak_titik(lokasi_misi[misi_tujuan][0], lokasi_misi[misi_tujuan][1], lokasi_misi[misi_tujuan][2], 
                current_position.pose.position.x, current_position.pose.position.y, current_position.pose.position.z) > 0.3){
                cmd_msg.linear.x = mypid_x.calculate(lokasi_misi[misi_tujuan][0], current_position.pose.position.x);
                cmd_msg.linear.y = mypid_y.calculate(lokasi_misi[misi_tujuan][1], current_position.pose.position.y);
                cmd_msg.linear.z = mypid_z.calculate(loksai_misi[misi_tujuan][2], current_position.pose.position.z);
                cmd_pub.publish(cmd_msg);

                ROS_INFO("Menuju misi %d", misi_tujuan);
                stay = ros::Time::now();
            }
            else if (ros::Time::now() - stay < ros::Duration(3.0)){
                cmd_msg.linear.x = mypid_x.calculate(lokasi_misi[misi_tujuan][0], current_position.pose.position.x);
                cmd_msg.linear.y = mypid_y.calculate(0, center_qr.offset_y);
                cmd_msg.linear.z = mypid_z.calculate(0, center_qr.offset_x);
                cmd_pub.publish(cmd_msg);
            }
            else if (ros::Time::now() - stay > ros::Duration(3.0)){
                //jatuhin box //ganti
                misi_tujuan += 1;
                meja_tujuan += 1;
                
            }
        }


        if (misi_tujuan == 4){
            cmd_msg.linear.x = mypid_x.calculate(0, current_position.pose.position.x);
            cmd_msg.linear.y = mypid_y.calculate(0, current_position.pose.position.y);
            cmd_msg.linear.z = mypid_z.calculate(0.5, current_position.pose.position.z);
            cmd_pub.publish(cmd_msg);

        }


        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("PILOOOOOOOOOOTTTTTTTTTTT");

    return 0;
}
