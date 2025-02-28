#pragma once

#include <iostream>

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