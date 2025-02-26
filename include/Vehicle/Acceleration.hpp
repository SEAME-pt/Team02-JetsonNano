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
    const float get_lateral() const;
    float get_mutable_lateral();
    void set_lateral(const float value);

    const float get_longitudinal() const;
    float get_mutable_longitudinal();
    void set_longitudinal(const float value);

    const float get_vertical() const;
    float get_mutable_vertical();
    void set_vertical(const float value);

    const std::string& get_description() const;
    std::string& get_mutable_description();
    void set_description(const std::string& value);

    Type get_type() const;
    void set_type(Type value);
};