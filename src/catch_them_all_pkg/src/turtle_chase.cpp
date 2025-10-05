// C++ Standard Library headers
#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"       // For controlling turtle velocity
#include "turtlesim/msg/pose.hpp"            // For subscribing to turtle pose
#include "turtlesim/srv/kill.hpp"            // Service to remove turtles
#include "rclcpp/rclcpp.hpp"                 // Core ROS2 functionality

#include "catch_them_all_interface/msg/turtle_info.hpp"  // Custom message for turtle info

using namespace std::chrono_literals;

/**
 * @struct TurtleData
 * @brief Holds the data for a single turtle.
 */
struct TurtleData {
    std::string name; ///< Turtle's name
    float x;          ///< X position
    float y;          ///< Y position
    float theta;      ///< Orientation in radians
};

/**
 * @class TurtleChaseNode
 * @brief Node for controlling turtle1 to chase and "kill" other turtles.
 *
 * Subscribes to:
 * - Turtle1's pose topic (`/turtle1/pose`)
 * - Positions of spawned turtles (`positions`)
 *
 * Publishes:
 * - Velocity commands for turtle1 (`/turtle1/cmd_vel`)
 *
 * Uses the `/kill` service to remove turtles when turtle1 is close enough.
 */
class TurtleChaseNode : public rclcpp::Node{
    public:
     /**
   * @brief Constructor for TurtleChaseNode.
   *
   * Initializes subscriptions, publishers, and service clients.
   */    
    TurtleChaseNode() : Node("turtle_chase") 
        { 
        // Subscribe to Turtle1 pose
        turtle1_sub_ = this->create_subscription<turtlesim::msg::Pose>(
            "/turtle1/pose",
            10,
            std::bind(&TurtleChaseNode::turtle1PoseCallback, this, std::placeholders::_1)
        );

        // Subscribe to all spawned turtles positions
        positions_sub_ = this->create_subscription<catch_them_all_interface::msg::TurtleInfo>(
            "positions",
            10,
            std::bind(&TurtleChaseNode::positionsCallback, this, std::placeholders::_1)
        );

        // Create client for killing turtles
        kill_client_ = this->create_client<turtlesim::srv::Kill>("/kill");

        // Publishes onto the /turtle1/cmd_vel topic
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);


        }
    private:
    rclcpp::Subscription<catch_them_all_interface::msg::TurtleInfo>::SharedPtr positions_sub_;  ///< Subscriber for spawned turtles
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr turtle1_sub_;                        ///< Subscriber for turtle1 pose
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;                      ///< Publisher for velocity commands
    rclcpp::Client<turtlesim::srv::Kill>::SharedPtr kill_client_;                               ///< Client to call `/kill` service

    std::vector<TurtleData> turtles_;  ///< List of all active turtles
    float turtle1_x_ = 0.0;            ///< Turtle1 X position
    float turtle1_y_ = 0.0;            ///< Turtle1 Y position
    float turtle1_theta_ = 0.0;        ///< Turtle1 orientation

     /**
   * @brief Callback for receiving positions of spawned turtles.
   * 
   * Updates the turtles_ vector or adds a new turtle if it does not exist.
   *
   * @param msg Shared pointer to TurtleInfo message
   */
    void positionsCallback(const catch_them_all_interface::msg::TurtleInfo::SharedPtr msg)
    {
        TurtleData t;
        t.name =msg->name;
        t.x = msg->x;
        t.y = msg->y;
        t.theta = msg->theta;

        // Check if the turtle already exists in the vector
        auto it = std::find_if(turtles_.begin(), turtles_.end(),
                  [&] (const TurtleData& td){return td.name == t.name;});

        if(it!=turtles_.end()){
            *it = t; //update existing turtle
        }else{
            turtles_.push_back(t); //add new
        }
    }

    /**
   * @brief Callback for receiving Turtle1's pose.
   * 
   * Finds the nearest turtle, moves turtle1 towards it, and kills it if close enough.
   *
   * @param msg Shared pointer to Pose message
   */
    void turtle1PoseCallback(const turtlesim::msg::Pose::SharedPtr msg)
    {
        turtle1_x_ = msg->x;
        turtle1_y_ = msg->y;
        turtle1_theta_ = msg->theta;

        if (turtles_.empty()) return;

        // Find nearest turtle
        float min_dist = std::numeric_limits<float>::max();
        size_t nearest_index = 0;

        for(size_t i = 0; i<turtles_.size();i++){
            //euclidean distance 
           float dx = turtle1_x_ - turtles_[i].x; 
           float dy = turtle1_y_ - turtles_[i].y;
           float dist = std::sqrt(dx*dx + dy*dy);

           if(dist<min_dist){
            min_dist =dist;
            nearest_index = i;
           }
        }

        if(nearest_index >= turtles_.size()) return; // Safety check

        // Kill turtle if close enough
        if(min_dist<0.5){
            auto kill_request = std::make_shared<turtlesim::srv::Kill::Request>();
            kill_request->name = turtles_[nearest_index].name;

            if (kill_client_->wait_for_service(1s)){
                kill_client_-> async_send_request(kill_request);
                RCLCPP_INFO(this->get_logger(), "Killed %s", turtles_[nearest_index].name.c_str());
            }
            turtles_.erase(turtles_.begin() + nearest_index);
        }else{
            // Move towards nearest turtle
            moveToTarget(turtles_[nearest_index].x, turtles_[nearest_index].y);
        }

    }


     /**
   * @brief Publishes a velocity command to move Turtle1 towards a target.
   *
   * Uses a proportional controller to calculate linear and angular velocity.
   *
   * @param target_x Target X position
   * @param target_y Target Y position
   */
    void moveToTarget(float target_x, float target_y)
    {
        geometry_msgs::msg::Twist cmd;

        float dx = target_x - turtle1_x_;
        float dy = target_y - turtle1_y_;
        float distance = std::sqrt(dx*dx + dy*dy);

        // Angle toward target
        float angle_to_target = std::atan2(dy, dx);
        float angle_diff = angle_to_target - turtle1_theta_;

        // Normalize angle_diff to [-pi, pi]
        while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
        while (angle_diff < -M_PI) angle_diff += 2 * M_PI;

        // Proportional controller 
        cmd.linear.x = std::min(1.5f, distance);     
        cmd.angular.z = std::max(std::min(4.0f * angle_diff, 2.0f), -2.0f);

        cmd_vel_pub_->publish(cmd);
    }
};

int main(int argc,char **argv){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<TurtleChaseNode>(); 
    rclcpp::spin(node); 
    rclcpp::shutdown();
    return 0;
}