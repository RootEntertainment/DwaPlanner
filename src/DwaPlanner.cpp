#include "dwa_navigation/DwaPlanner.h"
#include "dwa_navigation/ParamState.h"
#include "dwa_navigation/CustomState.h"

std_msgs::ColorRGBA DwaPlanner::Red = MakeColor(1, 0, 0, 1);
std_msgs::ColorRGBA DwaPlanner::Green = MakeColor(0, 1, 0, 1);
std_msgs::ColorRGBA DwaPlanner::Blue = MakeColor(0, 1, 1, 1);

DwaPlanner::DwaPlanner() : tf_listener_(tf_buffer_) {
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    
    // 参数加载
    private_nh.param("max_speed", max_speed_, 0.6);
    private_nh.param("min_speed", min_speed_, 0.0);
    private_nh.param("max_yaw_rate", max_yaw_rate_, 1.5);
    private_nh.param("max_accel", max_accel_, 0.3);
    private_nh.param("predict_time", predict_time_, 3.0);
    private_nh.param("goal_tolerance", goal_tolerance_, 0.2);
    private_nh.param("robot_radius", robot_radius_, 0.3);
    private_nh.param("resolution_v", resolution_v_, 0.05);
    private_nh.param("resolution_w", resolution_w_, 0.1);
    private_nh.param("window_size", window_size_, 0.5);
    private_nh.param("rate", rate_, 10.0);

    private_nh.param<std::string>("laser_topic", laser_topic, "/scan");
    private_nh.param<std::string>("cmd_topic", cmd_topic, "/cmd_vel");
    private_nh.param<std::string>("target_topic", target_topic, "/goal");
    private_nh.param<std::string>("odom_topic", odom_topic, "/odom");

    private_nh.param<std::string>("base_link", base_link, "base_link");
    private_nh.param<std::string>("laser_frame", laser_frame, "laser_frame");
    private_nh.param<std::string>("odom_frame", odom_frame, "odom_frame");
    
    dt_ = 1.0 / rate_;

    // 订阅发布
    laser_sub_ = nh.subscribe(laser_topic, 1, &DwaPlanner::laserCallback, this);
    goal_sub_ = nh.subscribe(target_topic, 1, &DwaPlanner::goalCallback, this);
    odom_sub_ = nh.subscribe(odom_topic, 1, &DwaPlanner::odomCallback, this);
    cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>(cmd_topic, 1);
    velocity_marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/velocity_sample",1);
    trajectory_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/best_trajectory",1);

    obstacle_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/obstacle_test",1);

    param_sm_ = new ParamStateMachine::StateMachine(this, AllStates, Conduits, Connections, Entrance);

    sensor_msgs::PointCloud2Modifier modifier(obstacles_);
    modifier.setPointCloud2FieldsByString(1, "xyz");
}

DwaPlanner::~DwaPlanner()
{
    delete param_sm_;
}

void DwaPlanner::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    //obt.points.clear();

    obstacles_.header.frame_id = odom_frame;
    obstacles_.header.stamp = ros::Time::now();

    geometry_msgs::TransformStamped transform;
    try {
        transform = tf_buffer_.lookupTransform(odom_frame, msg->header.frame_id,
                                             ros::Time(0));
    } catch (tf2::TransformException &ex) {
        ROS_WARN("TF error: %s", ex.what());
        return;
    }

    sensor_msgs::PointCloud2Modifier modifier(obstacles_);
    //modifier.resize(msg->ranges.size());
    modifier.clear();

    sensor_msgs::PointCloud2Iterator<float> iter_x(obstacles_, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(obstacles_, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(obstacles_, "z");
    double angle = msg->angle_min;
    int count = 0;
    for(auto& ray : msg->ranges){
        if(ray < msg->range_max - 0.01f){
            count ++;
            double x = ray * cos(angle);
            double y = ray * sin(angle);

            modifier.resize(count);

            auto iter = obstacles_.data.end();
            
            auto fiter = (float*)((iter-16).base());
            *fiter = x;
            *(fiter+1) = y;
            *(fiter+2) = 0;

            // *iter_x = x;
            // *iter_y = y;
            // *iter_z = 0;
            
            // ++iter_y;
            // ++iter_x;
            // ++iter_z;
        }
        angle += msg->angle_increment;
        //test
        // obt.points.emplace_back();
        // obt.points.back().x = x;
        // obt.points.back().y = y;
    }

    tf2::doTransform(obstacles_, obstacles_, transform);
    //obstacle_pub_.publish(obstacles_);
}


void DwaPlanner::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    geometry_msgs::TransformStamped transform;
    try {
        transform = tf_buffer_.lookupTransform(odom_frame, msg->header.frame_id,
                                             ros::Time(0));
    } catch (tf2::TransformException &ex) {
        ROS_WARN("TF error: %s", ex.what());
        return;
    }
    tf2::doTransform(*msg, current_goal_, transform);

    goal_received_ = true;
}

void DwaPlanner::run() {
    ros::Rate rate(rate_);
    while (ros::ok()) {
        param_sm_->Update(rate.expectedCycleTime().toSec());

        auto [a,b,c] = param_sm_->GetParams();
        ROS_INFO("current params:%f, %f, %f", a,b,c);

        if (goal_received_) {
            // 检查是否到达目标
            if (checkGoalReached()) {
                //ROS_INFO("Goal reached!");
                geometry_msgs::Twist cmd_vel;
                cmd_vel_pub_.publish(cmd_vel);
                goal_received_ = false;
            }else{
                geometry_msgs::Twist cmd_vel = computeBestVelocity();
                cmd_vel_pub_.publish(cmd_vel);
            }
        }
        
        ros::spinOnce();
        rate.sleep();
    }
}

geometry_msgs::Twist DwaPlanner::computeBestVelocity() {
    geometry_msgs::TransformStamped transform;
    try {
        transform = tf_buffer_.lookupTransform(base_link, odom_frame,
                                             ros::Time(0));
    } catch (tf2::TransformException &ex) {
        ROS_WARN("TF error: %s", ex.what());
        return geometry_msgs::Twist();
    }

    // 转换目标点到base_link坐标系
    geometry_msgs::PoseStamped goal_in_base;
    tf2::doTransform(current_goal_, goal_in_base, transform);

    //转换障碍到base_link坐标系
    sensor_msgs::PointCloud2 obstacles_in_base;
    tf2::doTransform(obstacles_, obstacles_in_base, transform);

    obstacles_in_base.header.frame_id = base_link;
    obstacle_pub_.publish(obstacles_in_base);

    std::vector<Velocity> velocities = generateVelocitySamples();
    
    for (auto& vel : velocities) {
        //evaluateVelocity(vel, goal_in_base);

        Trajectory trajectory = generateTrajectory(vel.v, vel.w);
        vel.dist_score = collision_curve(dist(trajectory, obstacles_in_base));
        vel.heading_score = head_curve(heading(trajectory, goal_in_base));
        //vel.velocity_score = velocity_curve(velocity(vel.v, vel.w));
    }

    evaluateVelocities(velocities);

    auto best = std::max_element(velocities.begin(), velocities.end(),
        [](const Velocity& a, const Velocity& b) { return a.score < b.score; });

    if(best != velocities.end()){
        publishVelocityMarker(velocities, best->score);
        publishTrajectorMarker(best->v, best->w);
    }

    geometry_msgs::Twist cmd_vel;
    if (best != velocities.end() && best->score > 0) {
        cmd_vel.linear.x = best->v;
        cmd_vel.angular.z = best->w;
    }
    return cmd_vel;
}

std::vector<DwaPlanner::Velocity> DwaPlanner::generateVelocitySamples() {
    std::vector<DwaPlanner::Velocity> samples;
    
    // 动态窗口计算
    double current_v = odom_data_.twist.twist.linear.x;
    double current_w = odom_data_.twist.twist.angular.z;
    
    double v_min = std::max(min_speed_, current_v - max_accel_ * window_size_);
    double v_max = std::min(max_speed_, current_v + max_accel_ * window_size_);
    double w_min = std::max(-max_yaw_rate_, current_w - max_accel_*2 / robot_radius_ * window_size_);
    double w_max = std::min(max_yaw_rate_, current_w + max_accel_ *2 / robot_radius_ * window_size_);

    // 速度采样
    for (double v = v_min; v <= v_max; v += resolution_v_) {
        for (double w = w_min; w <= w_max; w += resolution_w_) {
            samples.push_back({v, w});
        }
    }
    return samples;
}

DwaPlanner::Trajectory DwaPlanner::generateTrajectory(double v, double w){
    Trajectory trajectory;
    trajectory.push_back({0,0,0});
    for (double t = 0; t < predict_time_; t += dt_) {
        // 运动模型
        trajectory.push_back(differencialKineticModel(v, w, t));
    }
    return trajectory;
}

void DwaPlanner::evaluateVelocity(Velocity& vel, const geometry_msgs::PoseStamped& target) {

    // 目标评分
    // double dx = target.pose.position.x - x;
    // double dy = target.pose.position.y - y;
    // double dist = hypot(dx, dy);
    // double angle_diff = atan2(dy, dx) - theta;
    
    // double current_v = odom_data_.twist.twist.linear.x;
    // // 评分公式（可根据需要调整权重）
    // if(current_v < max_speed_ / 10){
    //         score = 0.5 * (1.0 / (dist + 1.0)) +  // 距离目标越近越好
    //         0.4 * cos(angle_diff) +       // 朝向目标方向
    //         (v < 0 ? (0.05 * abs(v)) : (0.15 * abs(v)));      // 鼓励move
    // }
    // else{
    //     score = 0.2 * (1.0 / (dist + 1.0)) +  // 距离目标越近越好
    //             0.8 * cos(angle_diff) +       // 朝向目标方向
    //             (v < 0 ? (0.2 * v) : 0);      // zuzhihoutui
    // }

    // score = 0.5 * (1.0 / (dist + 1.0)) +  // 距离目标越近越好
    //         0.2 * cos(angle_diff) +       // 朝向目标方向
    //         0.3 * v;    // 鼓励move
    // Trajectory trajectory = generateTrajectory(vel.v, vel.w);

    // vel.dist_score = collision_curve(dist(trajectory));
    // vel.heading_score = head_curve(heading(trajectory, target));
    // vel.velocity_score = velocity_curve(velocity(vel.v, vel.w));
}
void DwaPlanner::evaluateVelocities(std::vector<Velocity> &velocities)
{
    auto [kcollison, khead, kvel] = param_sm_->GetParams();
    

    double dist_sum = 0;
    double heading_sum = 0;
    double velocity_sum = 0;
    for (auto& vel : velocities) {
        if(vel.dist_score < 0) continue;
        dist_sum += vel.dist_score;
        heading_sum += vel.heading_score;

        vel.velocity_score = velocity(vel.v, vel.w, kvel);
        velocity_sum += vel.velocity_score;
    }
    
    for (auto& vel : velocities) {
        //vel.score = kcollison * vel.dist_score / dist_sum + khead * vel.heading_score / heading_sum + kvel * vel.velocity_score / velocity_sum;
        if(vel.dist_score < 0 || vel.heading_score < 0 || vel.velocity_score < 0) vel.score = -1e10;
        else vel.score = kcollison * vel.dist_score + khead * vel.heading_score + vel.velocity_score;
    }
}