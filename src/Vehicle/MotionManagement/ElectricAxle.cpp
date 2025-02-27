#include "ElectricAxle.hpp"

std::uint16_t ElectricAxleRow::get_rotational_speed() const
{
    return rotational_speed;
}

void ElectricAxleRow::set_rotational_speed(const std::uint16_t value)
{
    this->rotational_speed = value;
}

std::uint16_t ElectricAxleRow::get_rotational_speed_target() const
{
    return rotational_speed_target;
}

void ElectricAxleRow::set_rotational_speed_target(const std::uint16_t value)
{
    this->rotational_speed_target = value;
}

const ElectricAxleRow& ElectricAxle::get_row1() const
{
    return row1;
}
ElectricAxleRow& ElectricAxle::get_mutable_row1()
{
    return row1;
}
void ElectricAxle::set_row1(const ElectricAxleRow& value)
{
    this->row1 = value;
}

const ElectricAxleRow& ElectricAxle::get_row2() const
{
    return row2;
}
ElectricAxleRow& ElectricAxle::get_mutable_row2()
{
    return row2;
}
void ElectricAxle::set_row2(const ElectricAxleRow& value)
{
    this->row2 = value;
}

const std::string& ElectricAxle::get_description() const
{
    return description;
}
std::string& ElectricAxle::get_mutable_description()
{
    return description;
}
void ElectricAxle::set_description(const std::string& value)
{
    this->description = value;
}

Type ElectricAxle::get_type() const
{
    return type;
}
void ElectricAxle::set_type(Type value)
{
    this->type = value;
}