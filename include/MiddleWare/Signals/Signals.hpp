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
    std::shared_ptr<SensoringPublisher> publisher_;

  public:
    Signals(std::shared_ptr<SensoringPublisher> publisher);
    ~Signals();

    void init(const std::string& canDevice);
    void run();
};
