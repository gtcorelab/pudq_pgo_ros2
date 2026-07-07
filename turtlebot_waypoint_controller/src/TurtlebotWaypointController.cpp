#include "TurtlebotWaypointController.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

TurtlebotWaypointController::TurtlebotWaypointController() : Node("turtlebot_waypoint_controller_node") {
	RCLCPP_INFO(this->get_logger(), "Initializing Turtlebot Waypoint Controller Node");

    tb_name_ = std::string(this->get_namespace()).substr(1);

    //Enable bypass for start command
    this->declare_parameter("bypass_start", false);
    this->get_parameter("bypass_start", bypass_start_);

    if (bypass_start_) {
        RCLCPP_WARN(this->get_logger(), "Start bypass enabled.");
        running_ = true;
    }

	xy_threshold_ = 0.02;
	yaw_threshold_ = 0.2;

    std::vector<double>empty_vect;
	this->declare_parameter("waypoints_x", empty_vect);
	this->declare_parameter("waypoints_y", empty_vect);

    std::vector<double> waypoints_x, waypoints_y;
    this->get_parameter("waypoints_x", waypoints_x);
    this->get_parameter("waypoints_y", waypoints_y);

    //Check waypoint list size
    if (waypoints_x.size() != waypoints_y.size()) {
        RCLCPP_ERROR(this->get_logger(), "X and Y waypoints must be the same length. Exiting.");
        return;
    } else if (waypoints_x.size() == 0) {
        RCLCPP_ERROR(this->get_logger(), "No waypoints provided. Exiting.");
        return;
    }

    for (int i = 0; i < (int)waypoints_x.size(); i++) {
        geometry_msgs::msg::Point waypoint;

        waypoint.x = waypoints_x[i];
        waypoint.y = waypoints_y[i];
        waypoint.z = 0.0;

        waypoints_.push_back(waypoint);
    }

    RCLCPP_WARN(this->get_logger(), "Loaded %d waypoints.", (int)waypoints_.size());

    current_waypoint_ = 0;
    
    //Waypoint tf broadcaster
    waypoint_tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    waypoint_tf_.header.frame_id = std::string("map");
    waypoint_tf_.child_frame_id = tb_name_ + std::string("_waypoint");

    pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("pose", 10, std::bind(&TurtlebotWaypointController::pose_callback, this, _1));
    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    start_service_ = this->create_service<std_srvs::srv::Empty>("start", std::bind(&TurtlebotWaypointController::start, this, _1, _2));
}

double TurtlebotWaypointController::quat_to_yaw(geometry_msgs::msg::Quaternion q) {
    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    double yaw = std::atan2(siny_cosp, cosy_cosp);

    return yaw;
}

void TurtlebotWaypointController::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg) {

    //Return now if controller isn't running
    if (running_) {
        //Compute current yaw
        double yaw = quat_to_yaw(pose_msg->pose.orientation);

        Eigen::Vector2d x, x_d, v_x_d;
        x << pose_msg->pose.position.x, pose_msg->pose.position.y;
        x_d << waypoints_[current_waypoint_].x, waypoints_[current_waypoint_].y;

        double waypoint_dist = (x_d - x).norm();

        if (waypoint_dist < xy_threshold_) {
            RCLCPP_WARN(this->get_logger(), "WaypointController: Waypoint %d reached!", current_waypoint_);

            if (current_waypoint_ < waypoints_.size()-1) {
                current_waypoint_++;
                RCLCPP_WARN(this->get_logger(), "WaypointController: Loaded waypoint %d.", current_waypoint_);

                return;
            } else {
                RCLCPP_WARN(this->get_logger(), "WaypointController: All waypoints complete.");
                running_ = false;
                return;
            }
        }    

        v_x_d = (x_d - x)/waypoint_dist;
        double yaw_desired = std::atan2(v_x_d(1), v_x_d(0));
        double cos_dist = cos(yaw_desired - yaw);
        double sin_dist = sin(yaw_desired - yaw);
        double yaw_error = std::atan2(sin_dist, cos_dist);

        geometry_msgs::msg::Twist cmd_vel_msg;

        if (fabs(yaw_error) < yaw_threshold_) {
            cmd_vel_msg.linear.x = 0.22;
            cmd_vel_msg.angular.z = yaw_error;
        } else {
            cmd_vel_msg.linear.x = 0.0; 
            cmd_vel_msg.angular.z = yaw_error > 0 ? 1.0 : -1.0;
        }

        //Publish velocity command
        cmd_vel_publisher_->publish(cmd_vel_msg);

        //Broadcast current waypoint transform
        waypoint_tf_.header.stamp = this->get_clock()->now();;
        waypoint_tf_.transform.translation.x = waypoints_[current_waypoint_].x;
        waypoint_tf_.transform.translation.y = waypoints_[current_waypoint_].y;
        waypoint_tf_.transform.translation.z = waypoints_[current_waypoint_].z;
        waypoint_tf_broadcaster_->sendTransform(waypoint_tf_);
    } else {
        //Publish zeros if not running
        geometry_msgs::msg::Twist cmd_vel_msg;
        cmd_vel_publisher_->publish(cmd_vel_msg);
    }
}

void TurtlebotWaypointController::start(const std::shared_ptr<std_srvs::srv::Empty::Request> request, std::shared_ptr<std_srvs::srv::Empty::Response> response) {
    RCLCPP_WARN(this->get_logger(), "Starting trajectory");

    current_waypoint_ = 0;
    running_ = true;

    return;
}
