#pragma once

#include <iostream>

class Connectivity
{
  public:
    Connectivity()          = default;
    virtual ~Connectivity() = default;

  private:
    bool is_connectivity_available;

  public:
    bool get_is_connectivity_available() const;
    void set_is_connectivity_available(const bool value);
};
