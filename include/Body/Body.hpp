#pragma once

#include "Type.hpp"
#include "Lights.hpp"
#include <iostream>

class Body
{
  public:
    Body()          = default;
    virtual ~Body() = default;

  private:
    Lights lights;
    std::string description;
    Type type;

  public:
    const Lights& get_lights() const;
    Lights& get_mutable_lights();
    void set_lights(const Lights& value);

    const std::string& get_description() const;
    std::string& get_mutable_description();
    void set_description(const std::string& value);

    Type get_type() const;
    void set_type(Type value);
};