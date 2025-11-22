#include <iostream>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <offboard_rl/utils.h>
#include <Eigen/Dense>

using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class UnifiedPlanner : public rclcpp::Node
{
public:
    UnifiedPlanner() : Node("unified_planner")
    {
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

        // --- SUBSCRIPTIONS ---
        local_position_subscription_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position", qos,
            std::bind(&UnifiedPlanner::vehicle_local_position_callback, this, std::placeholders::_1));
        
        attitude_subscription_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>(
            "/fmu/out/vehicle_attitude", qos,
            std::bind(&UnifiedPlanner::vehicle_attitude_callback, this, std::placeholders::_1));

        // --- PUBLISHERS ---
        offboard_control_mode_publisher_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
        trajectory_setpoint_publisher_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
        vehicle_command_publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);

        // --- TIMERS ---
        timer_trajectory_publish_ = this->create_wall_timer(20ms, std::bind(&UnifiedPlanner::publish_trajectory_setpoint, this));
        timer_offboard_ = this->create_wall_timer(100ms, std::bind(&UnifiedPlanner::offboard_heartbeat, this));

        // --- SELEZIONE MODALITA' ---
        std::cout << "========================================" << std::endl;
        std::cout << "   ROBOTICS LAB - TRAJECTORY PLANNER" << std::endl;
        std::cout << "========================================" << std::endl;
        std::cout << "Select Operation Mode:" << std::endl;
        std::cout << " [1] Manual Input (Force Land Test - Point 2)" << std::endl;
        std::cout << " [2] Auto Trajectory (7 Waypoints - Point 3)" << std::endl;
        std::cout << "Choice: ";
        
        int choice;
        if (!(std::cin >> choice)) {
            choice = 1; // Default
            std::cin.clear();
            std::cin.ignore();
        }
        
        mode_ = choice;
        std::cin.ignore(); // Pulisce il buffer per i prossimi input

        if (mode_ == 2) {
            // Configurazione Waypoints per Modo 2 (Auto)
            waypoints_ = {
                {0.0, 0.0, -2.0, 0.0},     // 1. Decollo
                {5.0, 0.0, -2.0, 0.0},     // 2. Avanti
                {10.0, 5.0, -2.0, 1.57},   // 3. Diagonale + Yaw 90
                {5.0, 10.0, -4.0, 3.14},   // 4. Indietro/Alto + Yaw 180
                {0.0, 5.0, -2.0, -1.57},   // 5. Ritorno lato + Yaw -90
                {2.0, 2.0, -2.0, 0.0},     // 6. Avvicinamento
                {0.0, 0.0, -1.0, 0.0}      // 7. Atterraggio (quasi)
            };
            std::cout << "Mode 2 Selected: Press ENTER to start automatic mission..." << std::endl;
        } else {
            std::cout << "Mode 1 Selected: Manual input mode active." << std::endl;
        }

        // Avvio thread input
        keyboard_thread_ = std::thread(&UnifiedPlanner::input_listener, this);
    }

private:
    // --- ROS 2 Handles ---
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_subscription_;
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_subscription_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::TimerBase::SharedPtr timer_offboard_;
    rclcpp::TimerBase::SharedPtr timer_trajectory_publish_;
    std::thread keyboard_thread_;

    // --- Data ---
    VehicleLocalPosition current_position_{};
    VehicleAttitude current_attitude_{};
    
    // --- State Variables ---
    int mode_{1}; // 1 = Manual, 2 = Auto
    bool start_mission_{false}; // Per modo 2
    bool set_point_received_{false}; // Per modo 1
    bool offboard_active_{false};
    int offboard_counter_{0};

    // --- Variables for Mode 1 (Manual) ---
    Eigen::Vector4d manual_pos_i_, manual_pos_f_;
    double manual_T_{10.0}, manual_t_{0.0};
    bool manual_traj_computed_{false};
    Eigen::Vector<double, 6> manual_x_coeffs_;

    // --- Variables for Mode 2 (Auto) ---
    std::vector<Eigen::Vector4d> waypoints_;
    size_t current_wp_index_{0};
    double auto_t_{0.0};
    double segment_duration_{5.0};
    bool segment_init_{true};
    Eigen::Matrix<double, 6, 4> auto_coeffs_; 

    // --- Callbacks ---
    void vehicle_local_position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
        current_position_ = *msg;
    }
    void vehicle_attitude_callback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg) {
        current_attitude_ = *msg;
    }

    // --- Input Thread ---
    void input_listener()
    {
        if (mode_ == 2) {
            // MODO 2: Aspetta solo INVIO per partire
            std::cin.ignore();
            start_mission_ = true;
            std::cout << "Mission Started!" << std::endl;
            while(rclcpp::ok()) { std::this_thread::sleep_for(1s); }
        } 
        else {
            // MODO 1: Loop di input coordinate (Codice precedente)
            while (rclcpp::ok())
            {
                std::cout << "Enter setpoints as x y z yaw (meters, rads) & time (s): ";
                std::string line;
                std::getline(std::cin, line);
                std::istringstream iss(line);
                double in_x, in_y, in_z, in_yaw, in_T;

                if (!(iss >> in_x >> in_y >> in_z >> in_yaw >> in_T)) {
                    std::cout << "Invalid input." << std::endl;
                    continue;
                }

                // Aggiorna target
                manual_pos_f_ << in_x, in_y, -in_z, in_yaw; // Z invertita per input utente
                manual_T_ = in_T;

                // Aggiorna punto di partenza (Posizione Corrente)
                manual_pos_i_ << current_position_.x, current_position_.y, current_position_.z, 0.0;
                auto rpy = utilities::quatToRpy(Eigen::Vector4d(current_attitude_.q[0], current_attitude_.q[1], current_attitude_.q[2], current_attitude_.q[3]));
                manual_pos_i_(3) = rpy(2);

                // Reset logica traiettoria
                manual_t_ = 0.0;
                manual_traj_computed_ = false;
                set_point_received_ = true;
                
                // Reset Offboard per forzare il comando (utile per Retake Control)
                offboard_counter_ = 0;
                offboard_active_ = false;

                std::cout << "New setpoint received. Moving..." << std::endl;
            }
        }
    }

    // --- Offboard Heartbeat & Control ---
    void offboard_heartbeat()
    {
        bool active = (mode_ == 1) ? set_point_received_ : start_mission_;

        if (active)
        {
            if(offboard_counter_ == 10) {
                publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
                publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
                offboard_active_ = true;
            }
            OffboardControlMode msg{};
            msg.position = true;
            msg.velocity = (mode_ == 2); // Modo 2 usa velocità per fluidità
            msg.acceleration = (mode_ == 2); 
            msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
            offboard_control_mode_publisher_->publish(msg); 
            if (offboard_counter_ < 11) offboard_counter_++;
        }
    }

    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0)
    {
        VehicleCommand msg{};
        msg.param1 = param1; msg.param2 = param2; msg.command = command;
        msg.target_system = 1; msg.target_component = 1; msg.source_system = 1; msg.source_component = 1;
        msg.from_external = true;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        vehicle_command_publisher_->publish(msg);
    }

    // --- Main Loop ---
    void publish_trajectory_setpoint()
    {
        if (!offboard_active_) return;

        if (mode_ == 1 && set_point_received_) {
            run_manual_mode();
        } else if (mode_ == 2 && start_mission_) {
            run_auto_mode();
        }
    }

    // --- MODE 1 LOGIC (Point-to-Point Stop) ---
    void run_manual_mode()
    {
        // Logica originale "Stop at End"
        double current_t = (manual_t_ > manual_T_) ? manual_T_ : manual_t_;
        
        Vector4d e = manual_pos_f_ - manual_pos_i_;
        e(3) = utilities::angleError(manual_pos_f_(3), manual_pos_i_(3));
        double s_f = e.norm();

        if (!manual_traj_computed_) {
            Eigen::Matrix<double, 6, 6> A;
            Eigen::VectorXd b(6);
            b << 0,0,0, s_f, 0,0; // Start/End Vel/Acc = 0
            double T = manual_T_;
            A << 0,0,0,0,0,1,  0,0,0,0,1,0,  0,0,0,1,0,0,
                 pow(T,5),pow(T,4),pow(T,3),pow(T,2),T,1,
                 5*pow(T,4),4*pow(T,3),3*pow(T,2),2*T,1,0,
                 20*pow(T,3),12*pow(T,2),6*T,1,0,0;
            manual_x_coeffs_ = A.inverse() * b;
            manual_traj_computed_ = true;
        }

        double s = manual_x_coeffs_(0)*pow(current_t,5) + manual_x_coeffs_(1)*pow(current_t,4) + manual_x_coeffs_(2)*pow(current_t,3) + 
                   manual_x_coeffs_(3)*pow(current_t,2) + manual_x_coeffs_(4)*current_t + manual_x_coeffs_(5);
        
        // FIX: Sostituito operatore ternario complesso con if-else esplicito
        Eigen::Vector4d pos;
        if (s_f > 0.001) {
            pos = manual_pos_i_ + (s / s_f) * e;
        } else {
            pos = manual_pos_f_;
        }

        TrajectorySetpoint msg{};
        msg.position = {float(pos(0)), float(pos(1)), float(pos(2))};
        msg.yaw = float(pos(3));
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        trajectory_setpoint_publisher_->publish(msg);

        if (manual_t_ <= manual_T_) manual_t_ += 0.02;
    }

    // --- MODE 2 LOGIC (Continuous Fluid Path) ---
    void run_auto_mode()
    {
        if (current_wp_index_ >= waypoints_.size()) return;

        if (segment_init_) {
            std::cout << "Targeting WP " << current_wp_index_ + 1 << std::endl;
            compute_auto_coefficients();
            auto_t_ = 0.0;
            segment_init_ = false;
        }

        double t = auto_t_;
        Eigen::Vector4d pos, vel, acc;

        for (int i = 0; i < 4; i++) {
            pos(i) = auto_coeffs_(0,i)*pow(t,5) + auto_coeffs_(1,i)*pow(t,4) + auto_coeffs_(2,i)*pow(t,3) + auto_coeffs_(3,i)*pow(t,2) + auto_coeffs_(4,i)*t + auto_coeffs_(5,i);
            vel(i) = 5*auto_coeffs_(0,i)*pow(t,4) + 4*auto_coeffs_(1,i)*pow(t,3) + 3*auto_coeffs_(2,i)*pow(t,2) + 2*auto_coeffs_(3,i)*t + auto_coeffs_(4,i);
            acc(i) = 20*auto_coeffs_(0,i)*pow(t,3) + 12*auto_coeffs_(1,i)*pow(t,2) + 6*auto_coeffs_(2,i)*t + 2*auto_coeffs_(3,i);
        }

        TrajectorySetpoint msg{};
        msg.position = {float(pos(0)), float(pos(1)), float(pos(2))};
        msg.velocity = {float(vel(0)), float(vel(1)), float(vel(2))};
        msg.acceleration = {float(acc(0)), float(acc(1)), float(acc(2))};
        msg.yaw = float(pos(3));
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        trajectory_setpoint_publisher_->publish(msg);

        auto_t_ += 0.02;
        if (auto_t_ >= segment_duration_) {
            current_wp_index_++;
            segment_init_ = true;
        }
    }

    void compute_auto_coefficients()
    {
        Eigen::Vector4d pos_i, vel_i, acc_i, pos_f, vel_f, acc_f;

        if (current_wp_index_ == 0) {
            pos_i << current_position_.x, current_position_.y, current_position_.z, 0.0;
            auto rpy = utilities::quatToRpy(Eigen::Vector4d(current_attitude_.q[0], current_attitude_.q[1], current_attitude_.q[2], current_attitude_.q[3]));
            pos_i(3) = rpy(2);
            vel_i.setZero(); acc_i.setZero();
        } else {
            pos_i = waypoints_[current_wp_index_ - 1];
            vel_i = get_waypoint_velocity(current_wp_index_ - 1);
            acc_i.setZero();
        }

        pos_f = waypoints_[current_wp_index_];
        vel_f = get_waypoint_velocity(current_wp_index_);
        acc_f.setZero();

        double T = segment_duration_;
        Eigen::Matrix<double, 6, 6> A;
        A << 0,0,0,0,0,1, 0,0,0,0,1,0, 0,0,0,1,0,0,
             pow(T,5),pow(T,4),pow(T,3),pow(T,2),T,1,
             5*pow(T,4),4*pow(T,3),3*pow(T,2),2*T,1,0,
             20*pow(T,3),12*pow(T,2),6*T,1,0,0;
        
        Eigen::Matrix<double, 6, 6> A_inv = A.inverse();

        for (int axis = 0; axis < 4; axis++) {
            Eigen::VectorXd b(6);
            b << pos_i(axis), vel_i(axis), acc_i(axis), pos_f(axis), vel_f(axis), acc_f(axis);
            auto_coeffs_.col(axis) = A_inv * b;
        }
    }

    Eigen::Vector4d get_waypoint_velocity(size_t index)
    {
        if (index == 0 || index >= waypoints_.size() - 1) return Eigen::Vector4d::Zero();
        Eigen::Vector4d p_prev = waypoints_[index - 1];
        Eigen::Vector4d p_next = waypoints_[index + 1];
        Eigen::Vector4d v_dir = (p_next - p_prev).normalized();
        double speed = 1.0; 
        Eigen::Vector4d vel = v_dir * speed;
        vel(3) = 0.0; 
        return vel;
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UnifiedPlanner>());
    rclcpp::shutdown();
    return 0;
}
