#pragma once

#include "Type.hpp"
#include <iostream>

class Connectivity
{
  public:
    Connectivity()          = default;
    virtual ~Connectivity() = default;

  private:
    bool is_connectivity_available;
    std::string description;
    Type type;

  public:
    const bool get_is_connectivity_available() const;
    bool get_mutable_is_connectivity_available();
    void set_is_connectivity_available(const bool value);

    const std::string& get_description() const;
    std::string& get_mutable_description();
    void set_description(const std::string& value);

    Type get_type() const;
    void set_type(Type value);
};
