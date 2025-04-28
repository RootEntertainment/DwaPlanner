#pragma once

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h>
#include <cmath>
#include <vector>
#include <algorithm>

namespace ParamStateMachine{
    class StateMachine;
}


static std_msgs::ColorRGBA MakeColor(float r, float g, float b, float a){
    std_msgs::ColorRGBA color;
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = a;
    return color;
}

class DwaPlanner {
public:
    DwaPlanner();
    ~DwaPlanner();

    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        odom_data_ = *msg;
    }

    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

    void run();

private:
    struct Velocity {
        double v, w;
        double dist_score;
        double heading_score;
        double velocity_score;
        double score;
    };

    struct Pose{
        double x, y, w;
    };

    typedef std::vector<Pose> Trajectory;

    static std_msgs::ColorRGBA Red;
    static std_msgs::ColorRGBA Green;
    static std_msgs::ColorRGBA Blue;

    visualization_msgs::MarkerArray makeMarkerArray(){
        visualization_msgs::MarkerArray marker_array;
        visualization_msgs::Marker marker;

        marker.action = visualization_msgs::Marker::DELETEALL;
        marker_array.markers.push_back(marker);

        return marker_array;
    }

    visualization_msgs::Marker makeMarker(int32_t id, const Velocity& velocity, double highest){
        visualization_msgs::Marker marker;
        marker.action = visualization_msgs::Marker::ADD;
        
        marker.header.frame_id = base_link;
        marker.header.stamp = ros::Time::now();
        marker.ns = "velocity_markers";
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.text = std::to_string(velocity.score);
        marker.scale.x = 0.5;
        marker.scale.y = 0.5;
        marker.scale.z = 0.5;
        marker.id = id;

        Pose pose = differencialKineticModel(velocity.v, velocity.w, predict_time_);
        marker.pose.position.x = pose.x*10;
        marker.pose.position.y = pose.y*10;
        marker.pose.orientation.w = 1;
        if(velocity.score < -1e6){
            marker.color = Red;
        }
        else if(velocity.score == highest){
            marker.color = Green;
        }
        else{
            marker.color = Blue;
        }

        return marker;
    }

    void publishVelocityMarker(const std::vector<Velocity>& velocities, double highest){

        visualization_msgs::MarkerArray marker_array = makeMarkerArray();

        for(int i=0; i<velocities.size(); i++){
            // geometry_msgs::Point point;
            // point.x = pose.x;
            // point.y = pose.y;
            marker_array.markers.push_back(makeMarker(i, velocities[i], highest));
        }
        // for(auto& m : marker_array.markers){
        //     m.header.stamp = ros::Time::now();
        // }

        velocity_marker_pub_.publish(marker_array);
    }

    geometry_msgs::Twist computeBestVelocity();

public:
    bool checkGoalReached() {
        try {
            geometry_msgs::TransformStamped transform = tf_buffer_.lookupTransform(
                base_link, odom_frame, ros::Time(0));
            
            geometry_msgs::PoseStamped goal_in_base;
            tf2::doTransform(current_goal_, goal_in_base, transform);
            
            double dx = goal_in_base.pose.position.x;
            double dy = goal_in_base.pose.position.y;
            return (hypot(dx, dy) < goal_tolerance_);
        } catch (...) {
            return false;
        }
    }
private:
    std::vector<Velocity> generateVelocitySamples();

    // static Pose differencialKineticModel(Pose currentState, double v, double w, double dt){
    //     currentState.x += v * cos(currentState.w) * dt;
    //     currentState.y += v * sin(currentState.w) * dt;
    //     currentState.w += w * dt;
    //     return currentState;
    // }

    static Pose differencialKineticModel(double v, double w, double time){
        double x = v*sin(w*time)/w;
        double y = v*(1 - cos(w*time))/w;
        double theta = w*time;
        return Pose{x, y, theta};
    }

    Trajectory generateTrajectory(double v, double w);

    /*
    double minDist(const Pose& pose){
        double result = DBL_MAX;
        double angle = latest_scan_.angle_min;
        for(auto& ray : latest_scan_.ranges){
            double x = ray * sin(angle);
            double y = ray * cos(angle);
            result = std::min(result, hypot(pose.x - x, pose.y - y));
            angle += latest_scan_.angle_increment;
        }
        return result;
    }
    */
    double minDist(const Pose& pose, sensor_msgs::PointCloud2& obstacles_local){
        double result = DBL_MAX;

        sensor_msgs::PointCloud2Iterator<float> iter_x(obstacles_local, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(obstacles_local, "y");
        while(iter_x != iter_x.end() && iter_y != iter_y.end()){
            result = std::min(result, hypot(pose.x - *iter_x, pose.y - *iter_y));
            ++iter_x;
            ++iter_y;
        }

        // for(auto& p : obt.points){
        //     result = std::min(result, hypot(pose.x - p.x, pose.y - p.y));
        // }

        return result;
    }

    //路径上与障碍物的最小距离
    double dist(const Trajectory& trajectory, sensor_msgs::PointCloud2& obstacles_local){
        double dist = DBL_MAX;
        for(auto& pose : trajectory){
            dist = std::min(dist, minDist(pose, obstacles_local));
        }
        return dist;
    }

    //归一化曲线
    double collision_curve(double value){
        if(value < robot_radius_) return -1e10;
        if(value > 3*robot_radius_) return 1;
        return pow((value - robot_radius_) / (2*robot_radius_), 2);
        //return (value - robot_radius_) / (2*robot_radius_);
    }

    double heading(const Trajectory& trajectory, const geometry_msgs::PoseStamped& target){
        double dx = target.pose.position.x - trajectory.back().x;
        double dy = target.pose.position.y - trajectory.back().y;
        double angle = atan2(dy, dx);
        double delta_angle = angle - trajectory.back().w;
        return delta_angle;
    }

    //归一化曲线
    double head_curve(double value){
        //return pow(cos(value), 2);
        return (1+cos(value))/2;
        //return (M_PI - abs(value));
    }

    double velocity(double v, double w){
        //return (v / max_speed_ + abs(w / max_yaw_rate_)) / 2;
        return v;
    }

    //归一化曲线
    double velocity_curve(double value){
        if(value < 0) return 0;
        double current_v = odom_data_.twist.twist.linear.x;
        double v_min = std::max(0., current_v - max_accel_ * window_size_);
        double v_max = std::min(max_speed_, current_v + max_accel_ * window_size_);
        return (value-v_min) / (v_max-v_min);
    }

    void evaluateVelocity(Velocity& vel, const geometry_msgs::PoseStamped& target);
    void evaluateVelocities(std::vector<Velocity>& velocities);
    
    /*
    bool checkCollision(double x, double y) {
        if(latest_scan_.ranges.size() == 0) return false;
        // 转换到激光雷达坐标系（假设base_link与laser坐标系重合）
        double dist = hypot(x, y);
        double delta_angle = M_PI / 2;
        if(dist > robot_radius_) delta_angle = acos(robot_radius_/dist);
        double angle = atan2(y, x);

        // 查找对应的激光束
        int min_index = (angle - delta_angle - latest_scan_.angle_min) / latest_scan_.angle_increment;
        if(min_index < 0) min_index += latest_scan_.ranges.size();
        
        double angles = 0;
        for(int i = min_index; angles <= delta_angle * 2; i++, angles += latest_scan_.angle_increment){
            if (i >= latest_scan_.ranges.size()) i = 0;
            double theta = abs(angles - delta_angle);
            double h = dist*sin(theta);
            double length = dist * cos(theta) + sqrt(robot_radius_*robot_radius_-h*h);

            if (latest_scan_.ranges[i] < length) {
                return true;
            }
        }
        return false;
    }
    */
    
    // ROS成员
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    ros::Subscriber laser_sub_, goal_sub_, odom_sub_;
    ros::Publisher cmd_vel_pub_, velocity_marker_pub_, obstacle_pub_;
    
    // 算法参数
    double max_speed_, min_speed_, max_yaw_rate_, max_accel_;
    double predict_time_, goal_tolerance_, robot_radius_, window_size_, rate_;
    double dt_;
    double resolution_v_;
    double resolution_w_;

    //Topics
    std::string laser_topic;
    std::string cmd_topic;
    std::string target_topic;
    std::string odom_topic;

    //frames
    std::string base_link;
    std::string laser_frame;
    std::string odom_frame;
    
    // 数据缓存
    //sensor_msgs::LaserScan latest_scan_;
public:
    sensor_msgs::PointCloud obt;
    sensor_msgs::PointCloud2 obstacles_;
    geometry_msgs::PoseStamped current_goal_;
    nav_msgs::Odometry odom_data_;
    bool goal_received_ = false;

    //ParamState
    ParamStateMachine::StateMachine* param_sm_;

    double heading(){
        geometry_msgs::TransformStamped transform;
        try {
            transform = tf_buffer_.lookupTransform(base_link, odom_frame,
                                                ros::Time(0));
        } catch (tf2::TransformException &ex) {
            ROS_WARN("TF error: %s", ex.what());
            return 0;
        }

        // 转换目标点到base_link坐标系
        geometry_msgs::PoseStamped goal_in_base;
        tf2::doTransform(current_goal_, goal_in_base, transform);
        double dx = goal_in_base.pose.position.x;
        double dy = goal_in_base.pose.position.y;
        double angle = atan2(dy, dx);
        return angle;
    }
};

