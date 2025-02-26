#pragma once

#include "Type.hpp"
#include <iostream>

class Acceleration
{
  public:
    Acceleration()          = default;
    virtual ~Acceleration() = default;

  private:
    float lateral;
    float longitudinal;
    float vertical;
    std::string description;
    Type type;

  public:
    float get_lateral() const;
    void set_lateral(const float value);

    float get_longitudinal() const;
    void set_longitudinal(const float value);

    float get_vertical() const;
    void set_vertical(const float value);

    const std::string& get_description() const;
    std::string& get_mutable_description();
    void set_description(const std::string& value);

    Type get_type() const;
    void set_type(Type value);
};