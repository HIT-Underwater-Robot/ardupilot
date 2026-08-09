#pragma once

#include "Sub.h"

class Parameters;
class GCS_Sub;

class Mode
{
public:
    enum class Number : uint8_t {
        STABILIZE = 0,
        MANUAL = 19,
    };

    Mode(void);
    CLASS_NO_COPY(Mode);

    virtual bool init(bool ignore_checks) { return true; }
    virtual void run() = 0;
    virtual bool requires_GPS() const = 0;
    virtual bool requires_altitude() const = 0;
    virtual bool allows_arming(bool from_gcs) const = 0;
    virtual bool is_autopilot() const { return false; }
    virtual bool in_guided_mode() const { return false; }
    virtual const char *name() const = 0;
    virtual const char *name4() const = 0;
    virtual Mode::Number number() const = 0;

protected:
    Parameters &g;
    AP_AHRS &ahrs;
    AP_Motors6DOF &motors;
    RC_Channel *&channel_roll;
    RC_Channel *&channel_pitch;
    RC_Channel *&channel_throttle;
    RC_Channel *&channel_yaw;
    RC_Channel *&channel_forward;
    RC_Channel *&channel_lateral;
    AC_AttitudeControl_Sub *attitude_control;

};

class ModeManual : public Mode
{
public:
    using Mode::Mode;

    void run() override;
    bool init(bool ignore_checks) override;
    bool requires_GPS() const override { return false; }
    bool requires_altitude() const override { return false; }
    bool allows_arming(bool from_gcs) const override { return true; }
    bool is_autopilot() const override { return false; }

protected:
    const char *name() const override { return "Manual"; }
    const char *name4() const override { return "MANU"; }
    Mode::Number number() const override { return Mode::Number::MANUAL; }
};

class ModeStabilize : public Mode
{
public:
    using Mode::Mode;

    void run() override;
    bool init(bool ignore_checks) override;
    bool requires_GPS() const override { return false; }
    bool requires_altitude() const override { return false; }
    bool allows_arming(bool from_gcs) const override { return true; }
    bool is_autopilot() const override { return false; }

protected:
    const char *name() const override { return "Stabilize"; }
    const char *name4() const override { return "STAB"; }
    Mode::Number number() const override { return Mode::Number::STABILIZE; }
};
