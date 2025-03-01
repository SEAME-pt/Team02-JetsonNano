#pragma once

#include <fcntl.h>
#include <unistd.h>
#include <iostream>
#include "CAN.hpp"
#include <arpa/inet.h>
#include "VehicleFactory.hpp"
#include "VSSSubscriber.hpp"
#include "VSSQueryable.hpp"
#include "SensoringPublisher.hpp"

using namespace zenoh;

class Signals
{
  private:
    CAN* canBus;
    Vehicle vehicle_;
    std::unique_ptr<SensoringPublisher> publisher_;
    std::unique_ptr<VSSSubscriber> vss_subscriber_;

  public:
    Signals();
    ~Signals();

    void init(const std::string& canDevice);
    void run();
};
