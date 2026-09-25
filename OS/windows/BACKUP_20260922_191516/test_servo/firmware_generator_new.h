#pragma once
#include "kinematics.h"
#include <string>
#include <vector>

namespace r6dof {

// ============================================================
// NOTA FINALE CORRETTA - MAPPATURA da test_servo_finale / robot6dof_final_tinker.ino
// J1 -> GPIO 23   (270° 500-2450us)
// J2 -> GPIO 19   (180° 500-1800us) + GPIO 14 INVERSO (sync)
// J3 -> GPIO 25   (270°)
// J4 -> GPIO 26   (180°)
// J5 -> GPIO 18   (270°)
// J6 -> GPIO 13   (180°)
// UART Tinker: Serial2 GPIO16 RX / GPIO17 TX @115200 + Serial USB
// ============================================================

struct FirmwareConfig {
    std::string device_name = "robot6dof";
    int protocol_version = 1;
    int baud_rate = 115200;
    int servo_count = 6;
    int servo_hz = 50;
    int state_interval_ms = 100;
    std::array<int,6> servo_pins = {23, 19, 25, 26, 18, 13};
    int servo_pin_j2_inv = 14;
    std::array<float,6> servo_min_pw = {500, 500, 500, 500, 500, 500};
    std::array<float,6> servo_max_pw = {2450, 1800, 2450, 1800, 2450, 1800};
    std::array<float,6> servo_min_angle = {-135, -90, -135, -90, -135, -90};
    std::array<float,6> servo_max_angle = {135, 90, 135, 90, 135, 90};
    std::array<float,6> rest_pose = {0, 0, 0, 0, 0, 0}; // tinker coordinates -> HW 135,90,135,90,135,90
    bool invert_axes[6] = {false, false, false, false, false, false};
    std::string serial_port = "Serial2";
    int serial_rx_pin = 16;
    int serial_tx_pin = 17;
    // parametri S-Curve finale
    int velocita_master = 50;
    int velocita = 90;
    int velocita_j2 = 90;
    int velocita_rampa = 50;
    float potenza_j2 = 0.60;
};

std::string generate_firmware(const FirmwareConfig& cfg);
std::string generate_platformio_ini(const FirmwareConfig& cfg);
bool save_firmware(const std::string& dir, const FirmwareConfig& cfg);

}
