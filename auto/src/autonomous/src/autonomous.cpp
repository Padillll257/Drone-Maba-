#include "rclcpp/rclcpp.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "mavros_msgs/srv/command_tol.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;

class autoNode : public rclcpp::Node {
public:
    autoNode() : Node("autoL") {

        auto qos_sensor = rclcpp::QoS(10).best_effort();
        auto qos_reliable = rclcpp::QoS(10).reliable();

        pose_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/mavros/local_position/pose", qos_sensor,
            std::bind(&autoNode::pose_callback, this, std::placeholders::_1));
            
        state_sub = this->create_subscription<mavros_msgs::msg::State>(
            "/mavros/state", qos_sensor,
            std::bind(&autoNode::state_callback, this, std::placeholders::_1));

        point_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/mavros/setpoint_position/local", qos_reliable);

        arm_client = this->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
        state_client = this->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
        land_client = this->create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/land");
        takeoff_client = this->create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/takeoff");

        timer = this->create_wall_timer(100ms, std::bind(&autoNode::run, this));
    }

    float height = 1.5;
    float X = 0;
    float Y = 0;

private:
    enum State {
        SET_MODE,
        ARM,
        TAKEOFF,
        MOVE_X,
        WAIT_1,
        ROTATE,
        WAIT_2,
        MOVE_Y,
        LAND,
        DONE
    };

    State state = SET_MODE;

    mavros_msgs::msg::State cur_state;
    geometry_msgs::msg::PoseStamped cur_pose;
    geometry_msgs::msg::PoseStamped target_pose;
    rclcpp::Time state_time;

    float start_x, start_y;

    rclcpp::TimerBase::SharedPtr timer;

    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        cur_pose = *msg;
    }

    void state_callback(const mavros_msgs::msg::State::SharedPtr msg) {
        cur_state = *msg;
    }

    bool sampai(float x, float y, float z, float tol = 0.2) {
        return (fabs(cur_pose.pose.position.x - x) < tol &&
                fabs(cur_pose.pose.position.y - y) < tol &&
                fabs(cur_pose.pose.position.z - z) < tol);
    }

    void set_yaw(float yaw_rad) {
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw_rad);
        target_pose.pose.orientation = tf2::toMsg(q);
    }

    void publish_target(float x, float y, float z) {
        target_pose.header.stamp = this->now();
        target_pose.pose.position.x = x;
        target_pose.pose.position.y = y;
        target_pose.pose.position.z = z;
        point_pub->publish(target_pose);
    }

    bool wait(double seconds) {
        return (this->now() - state_time).seconds() > seconds;
    }

    void run() {
        switch(state) {

            //Guided
        case SET_MODE: {
            auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
            req->custom_mode = "GUIDED";
            state_client->async_send_request(req);

            if (cur_state.mode == "GUIDED") {
                RCLCPP_INFO(this->get_logger(), "GUIDED mode set");
                state = ARM;
            }
            break;
        }

        //Arming
        case ARM: {
            auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
            req->value = true;
            arm_client->async_send_request(req);

            if (cur_state.armed) {
                RCLCPP_INFO(this->get_logger(), "Armed");
                state = TAKEOFF;
            }
            break;
        }

        //Takeoff 
        case TAKEOFF: {
            auto req = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
            req->altitude = height;
            takeoff_client->async_send_request(req);

            if (cur_pose.pose.position.z >= height * 0.8) {
                start_x = cur_pose.pose.position.x;
                start_y = cur_pose.pose.position.y;

                RCLCPP_INFO(this->get_logger(), "Takeoff");
                state = MOVE_X;
            }
            break;
        }

        case MOVE_X: {
            float target_x = start_x + X;

            set_yaw(0); // hadap depan
            publish_target(target_x, start_y, height);

            if (sampai(target_x, start_y, height)) {
                RCLCPP_INFO(this->get_logger(), "Maju");
                state_time = this->now();
                state = WAIT_1;
            }
            break;
        }

        case WAIT_1: {
            if (wait(0.5)) {
                state = ROTATE;
            }
            break;
        }

        case ROTATE: {
            float yaw = (Y >= 0) ? -M_PI/2 : M_PI/2; // kanan negatif (ENU)
            set_yaw(yaw);
            publish_target(cur_pose.pose.position.x, cur_pose.pose.position.y, height);
            RCLCPP_INFO(this->get_logger(), "Muter");
            state_time = this->now();
            state = WAIT_2;
            break;
        }

        case WAIT_2: {
            if (wait(0.5)) {
                state = MOVE_Y;
            }
            break;
        }

        
        case MOVE_Y: {
            float target_y = start_y + Y;
            float target_x = start_x + X;
            publish_target(target_x, target_y, height);
            if (sampai(target_x, target_y, height)) {
                RCLCPP_INFO(this->get_logger(), "Belok");
                state = LAND;
            }
            break;
        }

        case LAND: {
            auto req = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
            land_client->async_send_request(req);
            if (!cur_state.armed) {
                RCLCPP_INFO(this->get_logger(), "Land");
                state = DONE;
            }
            break;
        }

        case DONE:
            RCLCPP_INFO(this->get_logger(), "Done");
            timer->cancel();
            break;
        }
    }

    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr point_pub;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_client;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr state_client;
    rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr land_client;
    rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client;
};

int main(int argc, char *argv[]) {
    float a, b, c;
    std::cout << "Masukkan ketinggian, maju X, kiri(+)/kanan(-) Y: ";
    std::cin >> a >> b >> c;
    rclcpp::init(argc, argv);
    auto node = std::make_shared<autoNode>();
    node->height = a;
    node->X  = b;
    node->Y  = c;
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}