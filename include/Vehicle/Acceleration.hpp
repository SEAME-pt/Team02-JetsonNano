#pragma once

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

  public:
    float get_lateral() const;
    void set_lateral(const float value);

    float get_longitudinal() const;
    void set_longitudinal(const float value);

    float get_vertical() const;
    void set_vertical(const float value);
};