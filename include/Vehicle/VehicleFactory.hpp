#pragma once

#include "Vehicle.hpp"
#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>

/**
 * @brief Factory class for creating preconfigured vehicle instances
 *
 * @details Creates and initializes Vehicle objects with default configurations
 * for:
 *          - Basic vehicle properties (dimensions, weight)
 *          - Powertrain settings
 *          - Body systems
 *          - ADAS configuration
 *          - Chassis setup
 *          - Exterior sensors
 *          - Connectivity systems
 *          - Motion management
 *
 * Default configurations include:
 * - Vehicle dimensions in millimeters
 * - Weight in kilograms
 * - Initial systems states (off/disabled)
 * - SAE Level 0 ADAS
 * - Economy performance mode
 * - Automatic transmission
 *
 * Usage:
 * @code{.cpp}
 * Vehicle vehicle = VehicleFactory::createDefaultVehicle();
 * @endcode
 *
 * @note All measurements follow SI units or VSS specifications
 * @see Vehicle
 */
class VehicleFactory
{
  public:
    /**
     * @brief Creates a new vehicle instance with default configuration
     * @return Vehicle Fully configured vehicle instance
     */
    static Vehicle createDefaultVehicle()
    {
        Vehicle vehicle;

        setupBasicProperties(vehicle);
        setupPowertrain(vehicle);
        setupBody(vehicle);
        setupADAS(vehicle);
        setupChassis(vehicle);
        setupExterior(vehicle);
        setupConnectivity(vehicle);
        setupMotion(vehicle);

        return vehicle;
    }

  private:
    /**
     * @brief Sets basic vehicle properties
     * @param vehicle Vehicle instance to configure
     * @details Configures dimensions, weight, and initial movement states
     */
    static void setupBasicProperties(Vehicle& vehicle)
    {
        vehicle.set_speed(0.0f);
        vehicle.set_is_moving(false);
        vehicle.set_average_speed(0.0f);
        vehicle.set_traveled_distance(0.0f);
        vehicle.set_traveled_distance_since_start(0.0f);
        vehicle.set_trip_duration(0.0f);
        vehicle.set_start_time(getCurrentTimestamp());

        // Set physical properties
        vehicle.set_height(1500);                 // mm
        vehicle.set_length(4500);                 // mm
        vehicle.set_turning_diameter(11000);      // mm
        vehicle.set_current_overall_weight(1500); // kg
    }

    /**
     * @brief Configures powertrain system
     * @param vehicle Vehicle instance to configure
     * @details Sets up transmission, motor, and drive modes
     */
    static void setupPowertrain(Vehicle& vehicle)
    {
        auto& powertrain = vehicle.get_mutable_powertrain();
        powertrain.set_range(0);
        powertrain.set_time_remaining(0);

        auto& transmission = powertrain.get_mutable_transmission();
        transmission.set_drive_type("REAR_WHEEL_DRIVE");
        transmission.set_travelled_distance(0.0f);
        transmission.set_current_gear(0);
        transmission.set_selected_gear(0);
        transmission.set_is_park_lock_engaged(true);
        transmission.set_performance_mode("ECONOMY");
        transmission.set_gear_change_mode("AUTOMATIC");

        auto& motor = powertrain.get_mutable_electric_motor();
        motor.set_max_power(100);
        motor.set_speed(0);
        motor.set_time_in_use(0);
    }

    /**
     * @brief Configures body systems
     * @param vehicle Vehicle instance to configure
     * @details Initializes lights and other body components
     */
    static void setupBody(Vehicle& vehicle)
    {
        auto& body = vehicle.get_mutable_body();

        auto& lights = body.get_mutable_lights();
        lights.get_mutable_beam_low().set_is_on(false);
        lights.get_mutable_beam_high().set_is_on(false);
        lights.get_mutable_running().set_is_on(false);
        lights.get_mutable_parking().set_is_on(false);
        lights.get_mutable_fog_rear().set_is_on(false);
        lights.get_mutable_fog_front().set_is_on(false);
        lights.get_mutable_brake().set_is_active(false);
        lights.get_mutable_hazard().set_is_signaling(false);
        lights.get_mutable_direction_indicator_left().set_is_signaling(false);
        lights.get_mutable_direction_indicator_right().set_is_signaling(false);
    }

    /**
     * @brief Configures ADAS system
     * @param vehicle Vehicle instance to configure
     * @details Sets autonomy levels and obstacle detection
     */
    static void setupADAS(Vehicle& vehicle)
    {
        auto& adas = vehicle.get_mutable_adas();
        adas.set_active_autonomy_level("SAE_0");
        adas.set_supported_autonomy_level("SAE_1");

        auto& obstacle_detection = adas.get_mutable_obstacle_detection();

        auto& front = obstacle_detection.get_mutable_front();
        front.set_is_enabled(true);

        auto& rear = obstacle_detection.get_mutable_rear();
        rear.set_is_enabled(false);
    }

    /**
     * @brief Configures chassis system
     * @param vehicle Vehicle instance to configure
     * @details Sets up axles, steering, brakes, and accelerator
     */
    static void setupChassis(Vehicle& vehicle)
    {
        auto& chassis = vehicle.get_mutable_chassis();
        chassis.set_wheelbase(10); // distancia entre eixos em mm
        chassis.set_axle_count(2);

        auto& axle = chassis.get_mutable_axle();

        auto& row1 = axle.get_mutable_row1();
        row1.set_tread_width(0);
        row1.set_track_width(0);
        row1.set_axle_width(0);
        row1.set_wheel_count(0);
        row1.set_wheel_diameter(0);
        row1.set_wheel_width(0);
        row1.set_steering_angle(0);
        row1.set_tire_diameter(0);
        row1.set_tire_width(0);
        row1.set_tire_aspect_ratio(0);

        auto& row2 = axle.get_mutable_row2();
        row2.set_tread_width(0);
        row2.set_track_width(0);
        row2.set_axle_width(0);
        row2.set_wheel_count(0);
        row2.set_wheel_diameter(0);
        row2.set_wheel_width(0);
        row2.set_steering_angle(0);
        row2.set_tire_diameter(0);
        row2.set_tire_width(0);
        row2.set_tire_aspect_ratio(0);

        // Setup steering
        auto& steering = chassis.get_mutable_steering_wheel();
        steering.set_angle(90);

        auto& accelerator = chassis.get_mutable_accelerator();
        accelerator.set_pedal_position(0);

        auto& brake = chassis.get_mutable_brake();
        brake.set_pedal_position(0);
        brake.set_is_driver_emergency_braking_detected(false);
    }

    /**
     * @brief Configures exterior sensors
     * @param vehicle Vehicle instance to configure
     * @details Initializes temperature, humidity, and light sensors
     */
    static void setupExterior(Vehicle& vehicle)
    {
        auto& exterior = vehicle.get_mutable_exterior();
        exterior.set_air_temperature(0);
        exterior.set_humidity(0);
        exterior.set_light_intensity(0);
    }

    /**
     * @brief Configures connectivity systems
     * @param vehicle Vehicle instance to configure
     * @details Sets up communication availability
     */
    static void setupConnectivity(Vehicle& vehicle)
    {
        auto& connectivity = vehicle.get_mutable_connectivity();
        connectivity.set_is_connectivity_available(true);
    }

    /**
     * @brief Configures motion management systems
     * @param vehicle Vehicle instance to configure
     * @details Sets up acceleration, angular velocity, and electric axles
     */
    static void setupMotion(Vehicle& vehicle)
    {
        // Setup acceleration
        auto& accel = vehicle.get_mutable_acceleration();
        accel.set_longitudinal(0);
        accel.set_lateral(0);
        accel.set_vertical(0);

        // Setup angular velocity
        auto& angular = vehicle.get_mutable_angular_velocity();
        angular.set_roll(0);
        angular.set_pitch(0);
        angular.set_yaw(0);

        // Setup motion management
        auto& motion = vehicle.get_mutable_motion_management();

        auto& electric_axle = motion.get_mutable_electric_axle();

        auto& row1 = electric_axle.get_mutable_row1();
        row1.set_rotational_speed(0);
        row1.set_rotational_speed_target(0);

        auto& row2 = electric_axle.get_mutable_row2();
        row2.set_rotational_speed(0);
        row2.set_rotational_speed_target(0);

        auto& steering = motion.get_mutable_steering();

        auto& steering_wheel = steering.get_mutable_steering_wheel();
        steering_wheel.set_angle(0);
        steering_wheel.set_angle_target(0);
    }

    /**
     * @brief Gets current timestamp in ISO format
     * @return std::string Formatted timestamp (YYYY-MM-DD HH:MM:SS)
     */
    static std::string getCurrentTimestamp()
    {
        auto now  = std::chrono::system_clock::now();
        auto time = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << std::put_time(std::localtime(&time), "%Y-%m-%d %H:%M:%S");
        return ss.str();
    }
};