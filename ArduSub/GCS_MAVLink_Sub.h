#pragma once

#include <GCS_MAVLink/GCS.h>

class GCS_MAVLINK_Sub : public GCS_MAVLINK
{
public:
    using GCS_MAVLINK::GCS_MAVLINK;

protected:
    void send_nav_controller_output() const override;
    void send_pid_tuning() override;
    void send_banner() override;
    void handle_manual_control_axes(const mavlink_manual_control_t &packet, uint32_t tnow) override;
    uint8_t send_available_mode(uint8_t index) const override;

private:
    void handle_message(const mavlink_message_t &msg) override;
    uint8_t base_mode() const override;
    MAV_STATE vehicle_system_status() const override;
    int16_t vfr_hud_throttle() const override;
};
