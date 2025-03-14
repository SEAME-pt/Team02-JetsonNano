#pragma once

#include <iostream>

/**
 * @brief Accelerator pedal management
 *
 * @details Controls and monitors accelerator functionality:
 *          - Pedal position (0-100%)
 *          - Position sensor integration
 *
 * @note 0% = Not depressed, 100% = Fully depressed
 * @note Implements VSS Chassis.Accelerator branch specification
 */
class Accelerator
{
  public:
    Accelerator()          = default;
    virtual ~Accelerator() = default;

  private:
    std::uint8_t pedal_position;

  public:
    std::uint8_t get_pedal_position() const;
    void set_pedal_position(const std::uint8_t value);
};