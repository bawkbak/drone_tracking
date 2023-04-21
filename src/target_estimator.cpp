#include <iostream>
#include <string> 

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
using namespace std; 


class Target{
    private:
        ros::NodeHandle n;
        ros::Publisher pub_target_pose;
        tf::TransformListener tf_listener;
        geometry_msgs::PoseStamped current_known_pose;
        geometry_msgs::PoseStamped current_estimated_pose;

        std::string m_target_tf = ""; 
        std::string m_robot_tf = ""; 
        int m_rate = 20;

    public:
        Target();
        void getParam();
        void estimateTarget();
        void pubGoal();
};

Target :: Target(){
    pub_target_pose = n.advertise<geometry_msgs::PoseStamped>("target/pose", 10);
}

void Target :: getParam(){
    string node_ns = ros::this_node::getName();

    n.getParam("/" + node_ns + "/robot_tf", m_robot_tf);
    n.getParam("/" + node_ns + "/target_tf", m_target_tf);
    n.getParam("/" + node_ns + "/rate", m_rate);
    return;
}

void Target :: estimateTarget(){
    ros::Rate rate(m_rate);

    tf::StampedTransform transform;
    // tf_listener.lookupTransform(m_robot_tf, m_target_tf,  ros::Time(0), transform);
    // tf_listener.lookupTransform("fcu", "tag_center",  ros::Time(0), transform);
    // try{
        
    // }
    // catch (tf::TransformException ex){
    //     ROS_ERROR("%s",ex.what());
    // }
    try{
        tf_listener.lookupTransform(m_robot_tf, m_target_tf, ros::Time(0), transform);
    }
    catch (tf::TransformException &ex) {
        // ROS_ERROR("%s",ex.what());
        // ros::Duration(1.0).sleep();
    }
    ROS_INFO("HEAD");
    cout << "x: " << transform.getOrigin().x() << endl;
    cout << "y: " << transform.getOrigin().y() << endl;
    cout << "z: " << transform.getOrigin().z() << endl;
    cout << "-------------------" << endl;
    tf::Quaternion q(
    transform.getRotation().x(),
    transform.getRotation().y(),
    transform.getRotation().z(),
    transform.getRotation().w());
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    cout << "r: " << roll * 180 / 3.14 << endl;
    cout << "p: " << pitch * 180 / 3.14 << endl;
    cout << "y: " << yaw * 180 / 3.14 << endl;

    rate.sleep();
    return;
}


int main(int argc, char **argv){
    ros::init(argc, argv, "visualize_path");
    Target tag;
    tag.getParam();
    while(ros::ok()){
        tag.estimateTarget();
        ros::spinOnce();
    }
    return 0;
}