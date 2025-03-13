#include <VSSQueryable.hpp>

VSSQueryable::VSSQueryable(Vehicle& vehicle,
                           std::shared_ptr<zenoh::Session> session)
    : vehicle_(vehicle)
{
    session_ = session;

    throttle_queryable.emplace(session_->declare_queryable(
        zenoh::KeyExpr("Vehicle/1/Powertrain/ElectricMotor/Speed"),
        [this](const zenoh::Query& query)
        {
            float speed = this->vehicle_.get_powertrain()
                              .get_electric_motor()
                              .get_speed();
            query.reply(query.get_keyexpr(), std::to_string(speed));
        },
        zenoh::closures::none, // on_drop callback
        zenoh::Session::QueryableOptions()));
    steering_queryable.emplace(session_->declare_queryable(
        zenoh::KeyExpr("Vehicle/1/Chassis/SteeringWheel/Angle"),
        [this](const zenoh::Query& query)
        {
            float angle =
                this->vehicle_.get_chassis().get_steering_wheel().get_angle();
            query.reply(query.get_keyexpr(), std::to_string(angle));
        },
        zenoh::closures::none, // on_drop callback
        zenoh::Session::QueryableOptions()));
    beamLow_queryable.emplace(session_->declare_queryable(
        zenoh::KeyExpr("Vehicle/1/Body/Lights/Beam/Low"),
        [this](const zenoh::Query& query)
        {
            bool isOn = this->vehicle_.get_body()
                            .get_lights()
                            .get_beam_low()
                            .get_is_on();
            query.reply(query.get_keyexpr(), std::to_string(isOn));
        },
        zenoh::closures::none, // on_drop callback
        zenoh::Session::QueryableOptions()));
    beamHigh_queryable.emplace(session_->declare_queryable(
        zenoh::KeyExpr("Vehicle/1/Body/Lights/Beam/High"),
        [this](const zenoh::Query& query)
        {
            bool isOn = this->vehicle_.get_body()
                            .get_lights()
                            .get_beam_high()
                            .get_is_on();
            query.reply(query.get_keyexpr(), std::to_string(isOn));
        },
        zenoh::closures::none, // on_drop callback
        zenoh::Session::QueryableOptions()));
    running_queryable.emplace(session_->declare_queryable(
        zenoh::KeyExpr("Vehicle/1/Body/Lights/Running"),
        [this](const zenoh::Query& query)
        {
            bool isOn = this->vehicle_.get_body()
                            .get_lights()
                            .get_running()
                            .get_is_on();
            query.reply(query.get_keyexpr(), std::to_string(isOn));
        },
        zenoh::closures::none, // on_drop callback
        zenoh::Session::QueryableOptions()));
    parking_queryable.emplace(session_->declare_queryable(
        zenoh::KeyExpr("Vehicle/1/Body/Lights/Parking"),
        [this](const zenoh::Query& query)
        {
            bool isOn = this->vehicle_.get_body()
                            .get_lights()
                            .get_parking()
                            .get_is_on();
            query.reply(query.get_keyexpr(), std::to_string(isOn));
        },
        zenoh::closures::none, // on_drop callback
        zenoh::Session::QueryableOptions()));
    fogRear_queryable.emplace(session_->declare_queryable(
        zenoh::KeyExpr("Vehicle/1/Body/Lights/Fog/Rear"),
        [this](const zenoh::Query& query)
        {
            bool isOn = this->vehicle_.get_body()
                            .get_lights()
                            .get_fog_rear()
                            .get_is_on();
            query.reply(query.get_keyexpr(), std::to_string(isOn));
        },
        zenoh::closures::none, // on_drop callback
        zenoh::Session::QueryableOptions()));
    fogFront_queryable.emplace(session_->declare_queryable(
        zenoh::KeyExpr("Vehicle/1/Body/Lights/Fog/Front"),
        [this](const zenoh::Query& query)
        {
            bool isOn = this->vehicle_.get_body()
                            .get_lights()
                            .get_fog_front()
                            .get_is_on();
            query.reply(query.get_keyexpr(), std::to_string(isOn));
        },
        zenoh::closures::none, // on_drop callback
        zenoh::Session::QueryableOptions()));
    brake_queryable.emplace(session_->declare_queryable(
        zenoh::KeyExpr("Vehicle/1/Body/Lights/Brake"),
        [this](const zenoh::Query& query)
        {
            bool isActive = this->vehicle_.get_body()
                                .get_lights()
                                .get_brake()
                                .get_is_active();
            query.reply(query.get_keyexpr(), std::to_string(isActive));
        },
        zenoh::closures::none, // on_drop callback
        zenoh::Session::QueryableOptions()));
    hazard_queryable.emplace(session_->declare_queryable(
        zenoh::KeyExpr("Vehicle/1/Body/Lights/Hazard"),
        [this](const zenoh::Query& query)
        {
            bool isSignaling = this->vehicle_.get_body()
                                   .get_lights()
                                   .get_hazard()
                                   .get_is_signaling();
            query.reply(query.get_keyexpr(), std::to_string(isSignaling));
        },
        zenoh::closures::none, // on_drop callback
        zenoh::Session::QueryableOptions()));
    directionIndicatorLeft_queryable.emplace(session_->declare_queryable(
        zenoh::KeyExpr("Vehicle/1/Body/Lights/DirectionIndicator/Left"),
        [this](const zenoh::Query& query)
        {
            bool isSignaling = this->vehicle_.get_body()
                                   .get_lights()
                                   .get_direction_indicator_left()
                                   .get_is_signaling();
            query.reply(query.get_keyexpr(), std::to_string(isSignaling));
        },
        zenoh::closures::none, // on_drop callback
        zenoh::Session::QueryableOptions()));
    directionIndicatorRight_queryable.emplace(session_->declare_queryable(
        zenoh::KeyExpr("Vehicle/1/Body/Lights/DirectionIndicator/Right"),
        [this](const zenoh::Query& query)
        {
            bool isSignaling = this->vehicle_.get_body()
                                   .get_lights()
                                   .get_direction_indicator_right()
                                   .get_is_signaling();
            query.reply(query.get_keyexpr(), std::to_string(isSignaling));
        },
        zenoh::closures::none, // on_drop callback
        zenoh::Session::QueryableOptions()));
}
