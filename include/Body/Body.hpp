#pragma once

#include "Lights.hpp"
#include <iostream>

/**
 * @brief Vehicle body systems management class
 *
 * @details Controls and monitors all body-related vehicle systems including:
 *          - Exterior lighting systems
 *          - Additional body functions (expandable)
 *
 * This class serves as the main interface for all body-related vehicle controls
 * and implements the VSS Body branch specification.
 *
 * @note Implements VSS Body branch specification
 * @see Lights
 */
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