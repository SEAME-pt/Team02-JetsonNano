#pragma once

#include "Type.hpp"
#include <iostream>

class Exterior
{
  public:
    Exterior()          = default;
    virtual ~Exterior() = default;

  private:
    float air_temperature;
    float humidity;
    float light_intensity;
    std::string description;
    Type type;

  public:
    float get_air_temperature() const;
    void set_air_temperature(const float value);

    float get_humidity() const;
    void set_humidity(const float value);

    float get_light_intensity() const;
    void set_light_intensity(const float value);

    const std::string& get_description() const;
    std::string& get_mutable_description();
    void set_description(const std::string& value);

    Type get_type() const;
    void set_type(Type value);
};