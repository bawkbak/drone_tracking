#include <iostream>
#include <string> 
#include <math.h>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <behavior_tree/behavior_tree.h>
using namespace std; 


class Target{
    private:
        ros::NodeHandle n;
        ros::Publisher pub_target_pose;
        ros::Publisher pub_error;
        ros::Subscriber sub_current_pose;
        tf::TransformListener tf_listener_goal;
        tf::TransformListener tf_listener_twist;
        geometry_msgs::PoseStamped current_known_pose;
        geometry_msgs::PoseStamped current_estimated_pose;

        std::string m_target_tf = ""; 
        std::string m_robot_tf = ""; 
        bool m_condition_status = false;
        int m_rate = 20;
        float m_offset_x, m_offset_y, m_offset_z, m_margin;
        float m_current_pose[7] = {0,0,0,0,0,0,0};

    public:
        bt::Condition condition;
        Target();
        void getParam();
        void positionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
        void estimateTarget();
        void conditionSet(bool state);
        void pubGoal(tf::StampedTransform input_tf);
        void pubErrorTwist();
};

Target :: Target() : condition("pose_in_margin"){
    pub_target_pose = n.advertise<geometry_msgs::PoseStamped>("target/pose", 10);
    pub_error = n.advertise<geometry_msgs::Twist>("target/error", 10);
    sub_current_pose = n.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 1,  &Target::positionCallback, this);
}

void Target :: getParam(){
    string node_ns = ros::this_node::getName();

    n.getParam("/" + node_ns + "/robot_tf", m_robot_tf);
    n.getParam("/" + node_ns + "/target_tf", m_target_tf);
    n.getParam("/" + node_ns + "/rate", m_rate);
    n.getParam("/" + node_ns + "/offset_x", m_offset_x);
    n.getParam("/" + node_ns + "/offset_y", m_offset_y);
    n.getParam("/" + node_ns + "/offset_z", m_offset_z);
    n.getParam("/" + node_ns + "/margin", m_margin);
    return;
}

void Target :: positionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    m_current_pose[0] = msg->pose.position.x;
    m_current_pose[1] = msg->pose.position.y;
    m_current_pose[2] = msg->pose.position.z;
    m_current_pose[3] = msg->pose.orientation.x;
    m_current_pose[4] = msg->pose.orientation.y;
    m_current_pose[5] = msg->pose.orientation.z;
    m_current_pose[6] = msg->pose.orientation.w;
    return;
}

void Target :: conditionSet(bool state){
    condition.set(state);
    condition.publish();
    return;
}

void Target :: estimateTarget(){
    ros::Rate rate(m_rate);

    tf::StampedTransform transform;

    bool t = true;
    try{
        tf_listener_goal.lookupTransform(m_robot_tf, m_target_tf, ros::Time(0), transform);
    }
    catch (tf::TransformException &ex) {
        // ROS_INFO("ERROR ");
        t = false;
    }
    
    if(t){
        pubGoal(transform);
        pubErrorTwist();
        conditionSet(m_condition_status);
    }
    
    rate.sleep();
    return;
}

void Target :: pubGoal(tf::StampedTransform input_tf){
    geometry_msgs::PoseStamped pub_msg_goal;

    double tag_roll, tag_pitch, tag_yaw, drone_roll, drone_pitch, drone_yaw;
    float ax, ay, az, aw;

    tf::Quaternion q_tag(
        input_tf.getRotation().x(),
        input_tf.getRotation().y(),
        input_tf.getRotation().z(),
        input_tf.getRotation().w()
    );
    tf::Matrix3x3 m_tag(q_tag);
    m_tag.getRPY(tag_roll, tag_pitch, tag_yaw);

    tf::Quaternion q_drone(
        m_current_pose[3],
        m_current_pose[4],
        m_current_pose[5],
        m_current_pose[6]
    );
    tf::Matrix3x3 m_drone(q_drone);
    m_drone.getRPY(drone_roll, drone_pitch, drone_yaw);

    tf::Quaternion q2;
    // if(abs(drone_yaw - tag_yaw) > 100){
    //     tag_yaw = 180 - tag_yaw;
    // }
    q2.setRPY(0, 0, tag_yaw);
    q2 = q2.normalize();
    ax = q2.getX();
    ay = q2.getY();
    az = q2.getZ();
    aw = q2.getW();

    pub_msg_goal.header.frame_id = "local_origin";
    // pub_msg_goal.pose.position.x = input_tf.getOrigin().x() + m_offset_x * cos(tag_yaw);
    // pub_msg_goal.pose.position.y = input_tf.getOrigin().y() + m_offset_y * sin(tag_yaw);
    // pub_msg_goal.pose.position.z = input_tf.getOrigin().z() + m_offset_z;

    // pub_msg_goal.pose.orientation.x = ax;
    // pub_msg_goal.pose.orientation.y = ay;
    // pub_msg_goal.pose.orientation.z = az;
    // pub_msg_goal.pose.orientation.w = aw;

    pub_msg_goal.pose.position.x = input_tf.getOrigin().x();
    pub_msg_goal.pose.position.y = input_tf.getOrigin().y();
    pub_msg_goal.pose.position.z = input_tf.getOrigin().z();

    pub_msg_goal.pose.orientation.x = input_tf.getRotation().x();
    pub_msg_goal.pose.orientation.y = input_tf.getRotation().y();
    pub_msg_goal.pose.orientation.z = input_tf.getRotation().z();
    pub_msg_goal.pose.orientation.w = input_tf.getRotation().w();

    // pub_msg_goal.pose.orientation.x = input_tf.getRotation().x();
    // pub_msg_goal.pose.orientation.y = input_tf.getRotation().y();
    // pub_msg_goal.pose.orientation.z = input_tf.getRotation().z();
    // pub_msg_goal.pose.orientation.w = input_tf.getRotation().w();

    // cout << "tag   x: " << input_tf.getOrigin().x() << endl;
    // cout << "tag   y: " << input_tf.getOrigin().y() << endl;
    // cout << "tag   z: " << input_tf.getOrigin().z() << endl;
    // cout << "-------------------" << endl;
    // cout << "drone  x: " << m_current_pose[0] << endl;
    // cout << "drone  y: " << m_current_pose[1] << endl;
    // cout << "drone  z: " << m_current_pose[2] << endl;
    // cout << "-------------------" << endl;
    // cout << "error x: " << pub_msg_goal.pose.position.x << endl;
    // cout << "error y: " << pub_msg_goal.pose.position.y << endl;
    // cout << "error z: " << pub_msg_goal.pose.position.z << endl;
    // cout << "-------------------" << endl;
    // cout << "drone angle: " << drone_yaw * 180 / 3.14 << endl;
    // cout << "tag   angle: " << tag_yaw * 180 / 3.14 << endl;
    // cout << "-------------------" << endl;

    pub_target_pose.publish(pub_msg_goal);
    float dis = sqrt(
        pow((m_current_pose[0] - input_tf.getOrigin().x()), 2) + 
        pow((m_current_pose[1] - input_tf.getOrigin().y()), 2) +
        pow((m_current_pose[2] - input_tf.getOrigin().z()), 2)
        );

    if(dis > m_margin / 1000){ m_condition_status = false;}
    else{ m_condition_status = true; }
    return;
}

void Target :: pubErrorTwist(){

    tf::StampedTransform transform;
    geometry_msgs::Twist pub_msg_error;
    bool t = true;
    try{
        tf_listener_twist.lookupTransform("/fcu", m_target_tf, ros::Time(0), transform);
    }
    catch (tf::TransformException &ex) {
        // ROS_INFO("ERROR ");
        t = false;
    }
    if(t){
        // cout << "error x:" << transform.getOrigin().x() << endl;
        // cout << "error y:" << transform.getOrigin().y() << endl;
        // cout << "error z:" << transform.getOrigin().z() << endl;
        // cout << "-------------------" << endl;
        pub_msg_error.linear.x = transform.getOrigin().x();
        pub_msg_error.linear.y = transform.getOrigin().y();
        pub_msg_error.linear.z = transform.getOrigin().z();
        pub_error.publish(pub_msg_error);
    }
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