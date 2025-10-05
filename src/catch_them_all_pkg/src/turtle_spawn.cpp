#include <cmath>
#include <random>
#include <string>

#include "rclcpp/rclcpp.hpp"                 // Core ROS2 functionality
#include "turtlesim/msg/pose.hpp"            // For subscribing to turtle pose
#include "turtlesim/srv/spawn.hpp"           // Service to spawn turtles

#include "catch_them_all_interface/msg/turtle_info.hpp"  // Custom message for turtle info




using namespace std::chrono_literals;

/**
 * @struct RandomPose
 * @brief Holds the position and orientation for a turtle.
 */
struct RandomPose {
    float x;
    float y;
    float theta;
};

/**
 * @class TurtleSpawnNode
 * @brief Spawns turtles at random positions in the turtlesim simulator and publishes their info.
 *
 * This node does three main things:
 * 1. Generates random positions and orientations for turtles.
 * 2. Publishes each turtle's information on the "positions" topic.
 * 3. Calls the /spawn service to spawn turtles in the simulator.
 */
class TurtleSpawnNode : public rclcpp::Node{ 
    public:
    /**
     * @brief Construct a TurtleSpawnNode.
     *
     * Initializes the spawn service client, publisher, timer, and random number generator.
     */
    TurtleSpawnNode() : Node("turtle_spawn") ,turtle_count(2)
    { 
        // Create service client for spawning turtles
        client_ = this->create_client<turtlesim::srv::Spawn>("/spawn");
        
        // Create publisher for turtle positions
        publisher_ = this->create_publisher<catch_them_all_interface::msg::TurtleInfo>("positions",10);
       
        // Create a timer that triggers every 1 second
        timer_ = this->create_wall_timer(1s,std::bind(&TurtleSpawnNode::sendPositionInfoandSpawn,this));
        
        // Seed the random number generator
        rng.seed(std::random_device{}());
    }
    private:
    rclcpp::Publisher<catch_them_all_interface::msg::TurtleInfo>::SharedPtr publisher_;///< Publisher for turtle info
    rclcpp::TimerBase::SharedPtr timer_;                                               ///< Timer to trigger spawning
    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr client_;                          ///< Client to call /spawn service
    int turtle_count;                                                                  ///< Client to call /spawn service
    std::mt19937 rng;                                                                  ///< Random number generator


    /**
   * @brief Generates a random position and orientation, publishes it, and spawns a new turtle.
   *
   * Called by the timer every second.
   */
    void sendPositionInfoandSpawn()
    {
        // Generate a random pose
        RandomPose pose = randomPointsGenerate();
        // Fill and publish TurtleInfo message
        catch_them_all_interface::msg::TurtleInfo msg;
        msg.name = "turtle" + std::to_string(turtle_count);
        msg.x = pose.x;
        msg.y = pose.y;
        msg.theta = pose.theta;
        publisher_->publish(msg);

        // Ensure the spawn service is available
        if (!client_->wait_for_service(1s)) {
            RCLCPP_WARN(this->get_logger(), "Spawn service not available");
            return;
        }
        // Prepare request to spawn the turtle
        auto request =std::make_shared<turtlesim::srv::Spawn::Request>();
        request->x = pose.x;
        request->y = pose.y;
        request->theta = pose.theta;
        request->name = "turtle" + std::to_string(turtle_count++);
        
        // Call the spawn service asynchronously
        auto future_result = client_->async_send_request(request);
    }

    /**
   * @brief Generates a random pose within safe boundaries in the turtlesim window.
   *
   * @return RandomPose struct with x, y, and theta
   */
     RandomPose randomPointsGenerate() {
        // Uniform distributions for x, y, and theta
        std::uniform_real_distribution<float> dist_xy(1.0, 10.0);      // Avoid edges
        std::uniform_real_distribution<float> dist_theta(0.0, 2.0 * M_PI);

        RandomPose pose;
        pose.x = dist_xy(rng);
        pose.y = dist_xy(rng);
        pose.theta = dist_theta(rng);

        return pose;
    }
};


int main(int argc,char **argv){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<TurtleSpawnNode>(); 
    rclcpp::spin(node); 
    rclcpp::shutdown();
    return 0;
}