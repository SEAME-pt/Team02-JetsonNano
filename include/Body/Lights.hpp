#pragma once

#include "Type.hpp"
#include <iostream>

class SignalingLights
{
  public:
    SignalingLights()          = default;
    virtual ~SignalingLights() = default;

  private:
    bool is_signaling;
    bool is_defect;

  public:
    const bool get_is_defect() const;
    bool get_mutable_is_defect();
    void set_is_defect(const bool value);

    const bool get_is_signaling() const;
    bool get_mutable_is_signaling();
    void set_is_signaling(const bool value);
};

class StaticLights
{
  public:
    StaticLights()          = default;
    virtual ~StaticLights() = default;

  private:
    bool is_on;
    bool is_defect;

  public:
    const bool get_is_defect() const;
    bool get_mutable_is_defect();
    void set_is_defect(const bool value);

    const bool get_is_on() const;
    bool get_mutable_is_on();
    void set_is_on(const bool value);
};

class BrakeLights
{
  public:
    BrakeLights()          = default;
    virtual ~BrakeLights() = default;

  private:
    bool is_active;
    bool is_defect;

  public:
    const bool get_is_defect() const;
    bool get_mutable_is_defect();
    void set_is_defect(const bool value);

    const bool get_is_active() const;
    bool get_mutable_is_active();
    void set_is_active(const bool value);
};

class Lights
{
  public:
    Lights()          = default;
    virtual ~Lights() = default;

  private:
    StaticLights beam;
    BrakeLights brake;
    SignalingLights direction_indicator;
    StaticLights fog;
    SignalingLights hazard;
    std::string light_switch;
    StaticLights parking;
    StaticLights running;
    std::string description;
    Type type;

  public:
    const StaticLights& get_beam() const;
    StaticLights& get_mutable_beam();
    void set_beam(const StaticLights& value);

    const BrakeLights& get_brake() const;
    BrakeLights& get_mutable_brake();
    void set_brake(const BrakeLights& value);

    const SignalingLights& get_direction_indicator() const;
    SignalingLights& get_mutable_direction_indicator();
    void set_direction_indicator(const SignalingLights& value);

    const StaticLights& get_fog() const;
    StaticLights& get_mutable_fog();
    void set_fog(const StaticLights& value);

    const SignalingLights& get_hazard() const;
    SignalingLights& get_mutable_hazard();
    void set_hazard(const SignalingLights& value);

    const std::string& get_light_switch() const;
    std::string& get_mutable_light_switch();
    void set_light_switch(const std::string& value);

    const StaticLights& get_parking() const;
    StaticLights& get_mutable_parking();
    void set_parking(const StaticLights& value);

    const StaticLights& get_running() const;
    StaticLights& get_mutable_running();
    void set_running(const StaticLights& value);

    const std::string& get_description() const;
    std::string& get_mutable_description();
    void set_description(const std::string& value);

    Type get_type() const;
    void set_type(Type value);
};