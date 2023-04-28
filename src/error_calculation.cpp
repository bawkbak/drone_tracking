#include <iostream>
#include <math.h>

#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/PoseStamped.h>
#include <gazebo_msgs/GetModelState.h>
#include <behavior_tree/behavior_tree.h>

using namespace std;

class Pose{
    private:
        ros::NodeHandle n;
        ros::ServiceClient srv_gazebo;
        ros::Subscriber sub_error;
        geometry_msgs::PoseStamped m_drone_gt;
        geometry_msgs::PoseStamped m_object_gt;

        float m_agent_error[6] = {0, 0, 0, 0, 0, 0};
    public:
        bt::Action action;
        Pose();
        bool srvCallObject();
        bool srvCallDrone();
        void errorCallback(const geometry_msgs::Twist::ConstPtr& msg);
        void quaternion2euler(float *roll, float *pitch, float *yaw, float x, float y, float z, float w);
        void actionSet(int state);
        void gtErrorEval();
        void agentErrorEval();
};

Pose :: Pose() : action("error_calculation"){
    srv_gazebo = n.serviceClient<gazebo_msgs::GetModelState>("gazebo/get_model_state");
    sub_error = n.subscribe<geometry_msgs::Twist>("target/error", 1,  &Pose::errorCallback, this);
}

bool Pose :: srvCallObject(){
    gazebo_msgs::GetModelState srv_msg;
    srv_msg.request.model_name = "visual_tag";
    // srv_msg.request.model_name = "drone";
    // srv_msg.request.relative_entity_name = "visual_tag::track";
    if(srv_gazebo.call(srv_msg)){ 
        m_object_gt.pose.position.x = srv_msg.response.pose.position.x;
        m_object_gt.pose.position.y = srv_msg.response.pose.position.y;
        m_object_gt.pose.position.z = srv_msg.response.pose.position.z;

        m_object_gt.pose.orientation.x = srv_msg.response.pose.orientation.x;
        m_object_gt.pose.orientation.y = srv_msg.response.pose.orientation.y;
        m_object_gt.pose.orientation.z = srv_msg.response.pose.orientation.z;
        m_object_gt.pose.orientation.w = srv_msg.response.pose.orientation.w;
        return true;
    }
    return false;
}

bool Pose :: srvCallDrone(){
    gazebo_msgs::GetModelState srv_msg;
    srv_msg.request.model_name = "drone";
     if(srv_gazebo.call(srv_msg)){ 
        m_drone_gt.pose.position.x = srv_msg.response.pose.position.x;
        m_drone_gt.pose.position.y = srv_msg.response.pose.position.y;
        m_drone_gt.pose.position.z = srv_msg.response.pose.position.z;

        m_drone_gt.pose.orientation.x = srv_msg.response.pose.orientation.x;
        m_drone_gt.pose.orientation.y = srv_msg.response.pose.orientation.y;
        m_drone_gt.pose.orientation.z = srv_msg.response.pose.orientation.z;
        m_drone_gt.pose.orientation.w = srv_msg.response.pose.orientation.w;
        return true;
    }
    return false;
}

void Pose :: errorCallback(const geometry_msgs::Twist::ConstPtr& msg){
    m_agent_error[0] = msg->linear.x;
    m_agent_error[1] = msg->linear.y;
    m_agent_error[2] = msg->linear.z;
    return;
}

void Pose :: quaternion2euler(float *roll, float *pitch, float *yaw, float x, float y, float z, float w){
    double tmp_r, tmp_p, tmp_y;
    tf::Quaternion q(x, y, z, w);
    tf::Matrix3x3 m(q);
    m.getRPY(tmp_r, tmp_p, tmp_y);
    *roll = (float)tmp_r;
    *pitch = (float)tmp_p;
    *yaw = (float)tmp_y;
    return;
}

void Pose :: actionSet(int state){
    switch(state){
        case 1:
            action.set_success();
            break;
        case 0:
            action.set_running();
            break;
        case -1:
            action.set_failure();
            break;
    }
    action.publish();
    return;
}

void Pose :: gtErrorEval(){
    // ros::Rate rate(30);

    bool state = 0;
    
    float error_x, error_y, error_z, error_distance, error_heading, tmp_angle;
    float object_angular_x, object_angular_y, object_angular_z, drone_angular_x, drone_angular_y, drone_angular_z;
    float distance_global_angle, object_alignment_global_angle, object_face_global_angle, error_related_angle;
    float distance_global_vector[2], object_face_global_vector[2];

    srvCallObject();
    srvCallDrone();

    // ROS_INFO("TAG");
    // cout << "x: " << m_object_gt.pose.position.x << " ";
    // cout << "y: " << m_object_gt.pose.position.y << " ";
    // cout << "z: " << m_object_gt.pose.position.z << " ";
    quaternion2euler(
        &object_angular_x, 
        &object_angular_y, 
        &object_angular_z, 
        m_object_gt.pose.orientation.x, 
        m_object_gt.pose.orientation.y, 
        m_object_gt.pose.orientation.z, 
        m_object_gt.pose.orientation.w
    );
    

    if(object_angular_z < 0){ tmp_angle = 3.14 * 2 + object_angular_z; }
    else{ tmp_angle = object_angular_z; }

    object_alignment_global_angle = tmp_angle - 1.57;
    object_face_global_angle = tmp_angle + 1.57;

    while(object_alignment_global_angle > 3.14){ object_alignment_global_angle = object_alignment_global_angle - 3.14 * 2; }
    while(object_face_global_angle > 3.14){ object_face_global_angle = object_face_global_angle - 3.14 * 2; }

    // cout << "R: " << object_angular_x * 180 / 3.14 << " ";   
    // cout << "P: " << object_angular_y * 180 / 3.14 << " ";
    // cout << "Y: " << object_angular_z * 180 / 3.14 << endl;
    // ROS_INFO("UAV");
    // cout << "x: " << m_drone_gt.pose.position.x << " ";
    // cout << "y: " << m_drone_gt.pose.position.y << " ";
    // cout << "z: " << m_drone_gt.pose.position.z << " ";
    quaternion2euler(
        &drone_angular_x, 
        &drone_angular_y, 
        &drone_angular_z, 
        m_drone_gt.pose.orientation.x, 
        m_drone_gt.pose.orientation.y, 
        m_drone_gt.pose.orientation.z, 
        m_drone_gt.pose.orientation.w
    );
    // cout << "R: " << drone_angular_x * 180 / 3.14 << " ";   
    // cout << "P: " << drone_angular_y * 180 / 3.14 << " ";
    // cout << "Y: " << drone_angular_z * 180 / 3.14 << endl;

    error_x = m_drone_gt.pose.position.x - m_object_gt.pose.position.x;
    error_y = m_drone_gt.pose.position.y - m_object_gt.pose.position.y;
    error_z = m_drone_gt.pose.position.z - m_object_gt.pose.position.z;
    error_distance = sqrt(
        pow(error_x, 2) + 
        pow(error_y, 2)
    );


    // cout << "error distance: " << error_distance << endl;
    // cout << "error angle   : " << (drone_angular_z - object_angular_z) * 180 / 3.14 << endl;


    if(m_drone_gt.pose.position.x >= m_object_gt.pose.position.x){
        // on same point
        if(m_object_gt.pose.position.y == m_drone_gt.pose.position.y && m_object_gt.pose.position.x == m_drone_gt.pose.position.x){
            distance_global_angle = -1000;
        }
        // on x axis
        else if(m_object_gt.pose.position.y == m_drone_gt.pose.position.y){
            distance_global_angle = 0;
        }
        // on y axis
        else if(m_object_gt.pose.position.x == m_drone_gt.pose.position.x){
            if(m_drone_gt.pose.position.y > m_object_gt.pose.position.y){
                distance_global_angle = 1.57;
            }else{
                distance_global_angle = -1.57;
            }
        }
        // on x+, y+
        else if(m_drone_gt.pose.position.y > m_object_gt.pose.position.y){
            // cout << "x+y+" << endl;
            distance_global_angle = atan(error_y / error_x);
        }
        // on x+, y-
        else if(m_drone_gt.pose.position.y < m_object_gt.pose.position.y){
            // cout << "x+y-" << endl;
            distance_global_angle = atan(error_y / error_x);
        }
    }else{
        // on x axis
        if(m_drone_gt.pose.position.y == m_object_gt.pose.position.y){
            distance_global_angle = -3.14;
        }
        // on x-, y+
        else if(m_drone_gt.pose.position.y > m_object_gt.pose.position.y){
            // cout << "x-y+" << endl;
            distance_global_angle = 3.14 + atan(error_y / error_x);
        }
        // on x-, y-
        else if(m_drone_gt.pose.position.y < m_object_gt.pose.position.y){
            // cout << "x-y-" << endl;
            distance_global_angle = -3.14 + atan(error_y / error_x);
        }
    }
    // cout << "error angle   : " << (object_angular_z - distance_global_angle) * 180 / 3.14 << endl; 

    
    // translate to vector
    distance_global_vector[0] = cos(distance_global_angle);
    distance_global_vector[1] = sin(distance_global_angle);
    
    object_face_global_vector[0] = cos(object_face_global_angle);
    object_face_global_vector[1] = sin(object_face_global_angle);
    // compare

    error_related_angle = acos((
        distance_global_vector[0] * object_face_global_vector[0] + 
        distance_global_vector[1] * object_face_global_vector[1] 
        ) / 
        sqrt(pow(distance_global_vector[0], 2) + pow(distance_global_vector[1], 2)) /
        sqrt(pow(object_face_global_vector[0], 2) + pow(object_face_global_vector[1], 2))
    );
    cout << "error: ground truth" << endl;

    cout << "x: " << (error_distance * cos(error_related_angle) - 1) * 1000
        << " y: " << (error_distance * sin(error_related_angle)) * 1000
        << " z: " << (error_z) * 1000 << endl;
    // rate.sleep();
    return;
}

void Pose :: agentErrorEval(){
    cout << "error: agent eval" << endl;

    cout << "x: " << m_agent_error[0] * 1000
        << " y: " << m_agent_error[1] * 1000
        << " z: " << m_agent_error[2] * 1000 << endl;
    

}

int main(int argc, char **argv){
    ros::init(argc, argv, "error");
    Pose error;
    while(ros::ok()){
        if(error.action.is_active() && error.action.active_has_changed()){
            // ROS_INFO("Action: Start");
            error.gtErrorEval();
            error.agentErrorEval();
        }
        else if(error.action.active_has_changed() && !(error.action.is_active())){
            // ROS_INFO("Action: Done");
            error.actionSet(1);
        }
        else if(error.action.is_active()){
            error.actionSet(0);
        }
        // error.gtErrorEval();
        ros::spinOnce();
    }
    return 0;
}