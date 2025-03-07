#pragma once

#include <iostream>

/**
 * @brief Signaling lights control class
 *
 * @details Controls lights that have a signaling function such as direction
 * indicators and hazard lights. Manages both the signaling state and defect
 * detection.
 *
 * @note Implements VSS SignalingLights specification
 */
class SignalingLights
{
  public:
    SignalingLights()          = default;
    virtual ~SignalingLights() = default;

  private:
    bool is_signaling;
    bool is_defect;

  public:
    bool get_is_defect() const;
    void set_is_defect(const bool value);

    bool get_is_signaling() const;
    void set_is_signaling(const bool value);
};

/**
 * @brief Static lights control class
 *
 * @details Controls lights that have a static on/off state such as beam lights,
 *          fog lights, parking lights and running lights. Includes defect
 * detection.
 *
 * @note Implements VSS StaticLights specification
 */
class StaticLights
{
  public:
    StaticLights()          = default;
    virtual ~StaticLights() = default;

  private:
    bool is_on;
    bool is_defect;

  public:
    bool get_is_defect() const;
    void set_is_defect(const bool value);

    bool get_is_on() const;
    void set_is_on(const bool value);
};

/**
 * @brief Brake lights control class
 *
 * @details Controls brake light functionality including:
 *          - Basic brake light activation
 *          - Adaptive emergency brake signaling
 *          - Defect monitoring
 *
 * States supported:
 * - INACTIVE: Lights are off
 * - ACTIVE: Lights are on
 * - ADAPTIVE: Emergency braking indication
 *
 * @note Implements VSS BrakeLights specification
 */
class BrakeLights
{
  public:
    BrakeLights()          = default;
    virtual ~BrakeLights() = default;

  private:
    bool is_active;
    bool is_defect;

  public:
    bool get_is_defect() const;
    void set_is_defect(const bool value);

    bool get_is_active() const;
    void set_is_active(const bool value);
};

/**
 * @brief Vehicle exterior lighting system management class
 *
 * @details Manages all exterior vehicle lights including:
 *          - Beam lights (high/low)
 *          - Running lights
 *          - Parking lights
 *          - Fog lights (front/rear)
 *          - Brake lights
 *          - Direction indicators
 *          - Hazard lights
 *
 * This class implements the VSS Body.Lights branch specification, providing
 * complete control over all exterior lighting functions.
 *
 * @note Implements VSS Body.Lights branch specification
 * @see SignalingLights
 * @see StaticLights
 * @see BrakeLights
 */
class Lights
{
  public:
    Lights()          = default;
    virtual ~Lights() = default;

  private:
    StaticLights beam_low;
    StaticLights beam_high;
    BrakeLights brake;
    SignalingLights direction_indicator_right;
    SignalingLights direction_indicator_left;
    StaticLights fog_rear;
    StaticLights fog_front;
    SignalingLights hazard;
    StaticLights parking;
    StaticLights running;

  public:
    const StaticLights& get_beam_low() const;
    StaticLights& get_mutable_beam_low();
    void set_beam_low(const StaticLights& value);

    const StaticLights& get_beam_high() const;
    StaticLights& get_mutable_beam_high();
    void set_beam_high(const StaticLights& value);

    const BrakeLights& get_brake() const;
    BrakeLights& get_mutable_brake();
    void set_brake(const BrakeLights& value);

    const SignalingLights& get_direction_indicator_left() const;
    SignalingLights& get_mutable_direction_indicator_left();
    void set_direction_indicator_left(const SignalingLights& value);

    const SignalingLights& get_direction_indicator_right() const;
    SignalingLights& get_mutable_direction_indicator_right();
    void set_direction_indicator_right(const SignalingLights& value);

    const StaticLights& get_fog_rear() const;
    StaticLights& get_mutable_fog_rear();
    void set_fog_rear(const StaticLights& value);

    const StaticLights& get_fog_front() const;
    StaticLights& get_mutable_fog_front();
    void set_fog_front(const StaticLights& value);

    const SignalingLights& get_hazard() const;
    SignalingLights& get_mutable_hazard();
    void set_hazard(const SignalingLights& value);

    const StaticLights& get_parking() const;
    StaticLights& get_mutable_parking();
    void set_parking(const StaticLights& value);

    const StaticLights& get_running() const;
    StaticLights& get_mutable_running();
    void set_running(const StaticLights& value);
};