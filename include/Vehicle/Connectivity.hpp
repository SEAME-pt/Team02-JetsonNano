#pragma once

#include <iostream>

/**
 * @brief Vehicle connectivity management system
 *
 * @details Manages vehicle-to-X (V2X) connectivity:
 *          - Connection availability monitoring
 *          - Cloud connectivity status
 *          - Service availability control
 *
 * @note Used for feature availability decisions
 * @note Implements VSS Vehicle.Connectivity branch specification
 */
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
