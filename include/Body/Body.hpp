#pragma once

#include "Lights.hpp"
#include <iostream>

class Body
{
  public:
    Body()          = default;
    virtual ~Body() = default;

  private:
    Lights lights;

  public:
    const Lights& get_lights() const;
    Lights& get_mutable_lights();
    void set_lights(const Lights& value);
};