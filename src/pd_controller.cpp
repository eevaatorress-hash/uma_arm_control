    /*

    Author: Juan M. Gandarias (http://jmgandarias.com)
    email: jmgandarias@uma.es


    This script uses a joint-space PD controller with non-linear dynamics compensation

    */

    #include <rclcpp/rclcpp.hpp>
    #include <sensor_msgs/msg/joint_state.hpp>
    #include <std_msgs/msg/float64_multi_array.hpp>
    #include <geometry_msgs/msg/wrench.hpp>
    #include <chrono>
    #include <Eigen/Dense>
    #include <cmath>

    class PDNode : public rclcpp::Node
    {
    public:
        PDNode()
            : Node("p_d_node"),
              joint_positions_(Eigen::VectorXd::Zero(2)),
              joint_velocities_(Eigen::VectorXd::Zero(2)),
              desired_joint_accel_(Eigen::VectorXd::Zero(2))
        {
            // Frequency initialization
            this->declare_parameter<double>("frequency", 1000.0);

            // Get frequency [Hz] parameter and compute period [s]
            double frequency = this->get_parameter("frequency").as_double();

            // Controller gains initialization
            zeta = 4.0; // Damping ratio
            wn = 0.5;   // Natural frequency

            KD << zeta*wn, 0,
                0, zeta*wn;

            KP << zeta*wn*wn, 0,
                0, zeta*wn*wn;

            // Reference position initialization
            q_ref << 0.0, 3.14 / 2;

            // Create subscription to joint_states
            subscription_joint_states_ = this->create_subscription<sensor_msgs::msg::JointState>(
                "joint_states", 1, std::bind(&PDNode::joint_states_callback, this, std::placeholders::_1));

            // Create publishers for joint acceleration
            publisher_desired_joint_accel_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("desired_joint_accelerations", 1);

            // Set the timer callback at a period (in milliseconds, multiply it by 1000)
            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(static_cast<int>(1000 / frequency)), std::bind(&PDNode::timer_callback, this));
        }

        // Timer callback - when there is a timer callback, computes the new joint acceleration, velocity and position and publishes them
        void timer_callback()
        {
            // Calculate desired acceleration
            desired_joint_accel_ = compute_pd_acceleration();

            // Publish data
            publish_data();
        }

    private:
        // joint_states subscription callback - when a new message arrives, updates the dynamics cancellation and publishes teh desired_joint_accel_
        void joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
        {
            // Assuming the joint names are "joint_1" and "joint_2"
            auto joint1_index = std::find(msg->name.begin(), msg->name.end(), "joint_1") - msg->name.begin();
            auto joint2_index = std::find(msg->name.begin(), msg->name.end(), "joint_2") - msg->name.begin();

            if (static_cast<std::vector<std::string>::size_type>(joint1_index) < msg->name.size() &&
                static_cast<std::vector<std::string>::size_type>(joint2_index) < msg->name.size())
            {
                joint_positions_(0) = msg->position[joint1_index];
                joint_positions_(1) = msg->position[joint2_index];
                joint_velocities_(0) = msg->velocity[joint1_index];
                joint_velocities_(1) = msg->velocity[joint2_index];
            }
        }

        // Method to calculate desired joint acceleration
        Eigen::VectorXd compute_pd_acceleration()
        {
            Eigen::Vector2d qddot_desired = - KD * joint_velocities_ + KP * ( q_ref  - joint_positions_);
            return qddot_desired;
        }

        // Method to publish the joint data
        void publish_data()
        {
            // publish joint accel
            auto desired_joint_accel_msg = std_msgs::msg::Float64MultiArray();
            desired_joint_accel_msg.data.assign(desired_joint_accel_.data(), desired_joint_accel_.data() + desired_joint_accel_.size());
            publisher_desired_joint_accel_->publish(desired_joint_accel_msg);
        }

        // Member variables
        // Publishers and subscribers
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_joint_states_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_desired_joint_accel_;
        rclcpp::TimerBase::SharedPtr timer_;

        // Joint variables
        Eigen::VectorXd joint_positions_;
        Eigen::VectorXd joint_velocities_;
        Eigen::VectorXd desired_joint_accel_;

        // Controller gains
        Eigen::Matrix2d KP;
        Eigen::Matrix2d KD;
        double zeta; // Damping ratio
        double wn;   // Natural frequency

        // Reference position
        Eigen::Vector2d q_ref;
    };

    int main(int argc, char *argv[])
    {
        rclcpp::init(argc, argv);
        auto node = std::make_shared<PDNode>();
        rclcpp::spin(node);
        rclcpp::shutdown();
        return 0;
    }