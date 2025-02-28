#pragma once

#include "Vehicle.hpp"
#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>

class VehicleFactory
{
  public:
    static Vehicle createDefaultVehicle()
    {
        Vehicle vehicle;

        // Set basic vehicle properties
        setupBasicProperties(vehicle);

        // Set all major components
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

    static void setupPowertrain(Vehicle& vehicle)
    {
        auto& powertrain = vehicle.get_mutable_powertrain();
        powertrain.set_range(0);
        powertrain.set_time_remaining(0);

        auto& motor = powertrain.get_mutable_electric_motor();
        motor.set_speed(0);
        motor.set_max_power(100);
        motor.set_time_in_use(0);

        auto& transmission = powertrain.get_mutable_transmission();
        transmission.set_drive_type("REAR_WHEEL_DRIVE");
        transmission.set_travelled_distance(0.0f);
        transmission.set_current_gear(0);
        transmission.set_selected_gear(0);
        transmission.set_is_park_lock_engaged(true);
        transmission.set_performance_mode("ECONOMY");
        transmission.set_gear_change_mode("AUTOMATIC");
    }

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

        auto& left = obstacle_detection.get_mutable_left();
        left.set_is_enabled(false);

        auto& right = obstacle_detection.get_mutable_right();
        right.set_is_enabled(false);

        auto& center = obstacle_detection.get_mutable_center();
        center.set_is_enabled(false);
    }

    static void setupChassis(Vehicle& vehicle)
    {
        auto& chassis = vehicle.get_mutable_chassis();
        chassis.set_axle_count(2);
        chassis.set_wheelbase(10); // distancia entre eixos em mm

        auto& axle = chassis.get_mutable_axle();

        auto& row1 = axle.get_mutable_row1();

        auto& row2 = axle.get_mutable_row2();

        // Setup steering
        auto& steering = chassis.get_mutable_steering_wheel();
        steering.set_angle(90);

        auto& accelerator = chassis.get_mutable_accelerator();

        auto& brake = chassis.get_mutable_brake();
    }

    static void setupExterior(Vehicle& vehicle)
    {
        auto& exterior = vehicle.get_mutable_exterior();
    }

    static void setupConnectivity(Vehicle& vehicle)
    {
        auto& connectivity = vehicle.get_mutable_connectivity();
    }

    static void setupMotion(Vehicle& vehicle)
    {
        // Setup acceleration
        auto& accel = vehicle.get_mutable_acceleration();

        // Setup angular velocity
        auto& angular = vehicle.get_mutable_angular_velocity();

        // Setup motion management
        auto& motion = vehicle.get_mutable_motion_management();
    }

    static std::string getCurrentTimestamp()
    {
        auto now  = std::chrono::system_clock::now();
        auto time = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << std::put_time(std::localtime(&time), "%Y-%m-%d %H:%M:%S");
        return ss.str();
    }
};