#pragma once

#include "Type.hpp"
#include <iostream>

class Accelerator
{
  public:
    Accelerator()          = default;
    virtual ~Accelerator() = default;

  private:
    std::uint8_t pedal_position;
    std::string description;
    Type type;

  public:
    const std::uint8_t get_pedal_position() const;
    std::uint8_t get_mutable_pedal_position();
    void set_pedal_position(const std::uint8_t value);

    const std::string& get_description() const;
    std::string& get_mutable_description();
    void set_description(const std::string& value);

    Type get_type() const;
    void set_type(Type value);
};