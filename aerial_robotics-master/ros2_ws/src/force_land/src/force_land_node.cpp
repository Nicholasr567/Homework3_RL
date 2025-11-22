#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>

using namespace std::chrono_literals;

class ForceLand : public rclcpp::Node
{
public:
	ForceLand() : Node("force_land"), need_land(false), land_triggered(false)
	{
		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

		subscription_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>("/fmu/out/vehicle_local_position_v1",
																					   qos, std::bind(&ForceLand::height_callback, this, std::placeholders::_1));

		// Iscrizione opzionale per debug o logiche future
		land_detected_sub_ = this->create_subscription<px4_msgs::msg::VehicleLandDetected>("/fmu/out/vehicle_land_detected",
																						   qos, std::bind(&ForceLand::land_detected_callback, this, std::placeholders::_1));

		publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);

		timer_ = this->create_wall_timer(100ms, std::bind(&ForceLand::activate_switch, this));
	}

private:
	rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr subscription_;
	rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr land_detected_sub_;
	rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr publisher_;

	rclcpp::TimerBase::SharedPtr timer_;

	bool need_land;
	bool land_triggered; // Flag per evitare lo spam del comando

	void height_callback(const px4_msgs::msg::VehicleLocalPosition::UniquePtr msg)
	{
		// PX4 usa NED (Z negativo in alto). Convertiamo in positivo per comodità.
		float altitude = -msg->z;

		// Logica Rising Edge con Isteresi
		if(altitude > 20.0)
		{
			// Se siamo sopra i 20m e NON abbiamo ancora attivato l'atterraggio
			if (!land_triggered) {
				std::cout << "Altitude threshold exceeded (" << altitude << "m). Triggering Force Land." << std::endl;
				need_land = true;
			}
		}
		else if (altitude < 19.0)
		{
			// Reset del trigger solo se scendiamo sotto i 19m (isteresi per sicurezza)
			if (land_triggered) {
				std::cout << "Altitude safe (" << altitude << "m). Resetting trigger." << std::endl;
				land_triggered = false;
			}
		}
	}

	void land_detected_callback(const px4_msgs::msg::VehicleLandDetected::UniquePtr msg)
	{
		// Callback vuota per ora, utile se si vogliono aggiungere logiche basate sullo stato "landed"
		(void)msg; // Evita warning unused parameter
	}

	void activate_switch()
	{
		if(need_land)
		{
			std::cout << "Sending LAND Command..." << std::endl;
			auto command = px4_msgs::msg::VehicleCommand();
			command.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND;
			command.target_system = 1;
			command.target_component = 1;
			command.source_system = 1;
			command.source_component = 1;
			command.from_external = true;
			command.timestamp = this->get_clock()->now().nanoseconds() / 1000;

			this->publisher_->publish(command);

			// Importante: Segniamo che abbiamo attivato l'atterraggio
			// e spegniamo la richiesta immediata.
			land_triggered = true;
			need_land = false;
		}
	}
};

int main(int argc, char *argv[])
{
	std::cout << "Starting Force Land node..." << std::endl;
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ForceLand>());
	rclcpp::shutdown();
	return 0;
}
