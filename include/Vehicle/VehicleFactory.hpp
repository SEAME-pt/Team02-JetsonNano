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
        vehicle.set_type(Type::BRANCH);
        vehicle.set_description("Default Vehicle Configuration");
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
        powertrain.set_type(Type::BRANCH);
        powertrain.set_description("");

        // Setup electric motor
        auto& motor = powertrain.get_mutable_electric_motor();
        motor.set_type(Type::BRANCH);
        motor.set_speed(0);

        auto& transmission = powertrain.get_mutable_transmission();
        transmission.set_type(Type::BRANCH);
    }

    static void setupBody(Vehicle& vehicle)
    {
        auto& body = vehicle.get_mutable_body();
        body.set_type(Type::BRANCH);

        // Setup lights
        auto& lights = body.get_mutable_lights();
        lights.set_type(Type::BRANCH);
    }

    static void setupADAS(Vehicle& vehicle)
    {
        auto& adas = vehicle.get_mutable_adas();
        adas.set_type(Type::BRANCH);
        adas.set_description("Advanced Driver Assistance System");

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
        chassis.set_type(Type::BRANCH);

        // Setup steering
        auto& steering = chassis.get_mutable_steering_wheel();
        steering.set_type(Type::ACTUATOR);
        steering.set_angle(0);
    }

    static void setupExterior(Vehicle& vehicle)
    {
        auto& exterior = vehicle.get_mutable_exterior();
        exterior.set_type(Type::BRANCH);
    }

    static void setupConnectivity(Vehicle& vehicle)
    {
        auto& connectivity = vehicle.get_mutable_connectivity();
        connectivity.set_type(Type::BRANCH);
    }

    static void setupMotion(Vehicle& vehicle)
    {
        // Setup acceleration
        auto& accel = vehicle.get_mutable_acceleration();
        accel.set_type(Type::BRANCH);

        // Setup angular velocity
        auto& angular = vehicle.get_mutable_angular_velocity();
        angular.set_type(Type::BRANCH);

        // Setup motion management
        auto& motion = vehicle.get_mutable_motion_management();
        motion.set_type(Type::BRANCH);
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