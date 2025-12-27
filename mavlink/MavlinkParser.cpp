#include "MavlinkParser.h"

// Функции обработки различных типов MAVLink сообщений

void MavlinkParser::HEARTBEAT_func(mavlink_message_t msg)
{
    mavlink_heartbeat_t heartbeat;
    mavlink_msg_heartbeat_decode(&msg, &heartbeat);

    std::lock_guard<std::mutex> lock(telemetry_state_formater.mtx);
    telemetry_state_formater.heartbeat.has_value = true;

    telemetry_state_formater.heartbeat.last_update_point = std::chrono::steady_clock::now();
    telemetry_state_formater.heartbeat.text =
        "type:" + std::to_string(heartbeat.type) +
        " autopilot:" + std::to_string(heartbeat.autopilot) +
        " base_mode:" + std::to_string(heartbeat.base_mode) +
        " custom_mode:" + std::to_string(heartbeat.custom_mode) +
        " system_status:" + std::to_string(heartbeat.system_status) +
        " mavlink_version:" + std::to_string(heartbeat.mavlink_version);
}

void MavlinkParser::SYS_STATUS_func(mavlink_message_t msg)
{
    mavlink_sys_status_t mavlink_status_info;
    mavlink_msg_sys_status_decode(&msg, &mavlink_status_info);
    
    std::lock_guard<std::mutex> lock(telemetry_state_formater.mtx);
    telemetry_state_formater.sys_status.has_value = true;
    telemetry_state_formater.sys_status.last_update_point = std::chrono::steady_clock::now();
    
    telemetry_state_formater.sys_status.text =
        "voltage_battery:" + std::to_string(mavlink_status_info.voltage_battery) +
        " current_battery:%" + std::to_string(mavlink_status_info.current_battery) +
        " battery_remaining:" + std::to_string(mavlink_status_info.battery_remaining) +
        " onboard_control_present:" + std::to_string(mavlink_status_info.onboard_control_sensors_present) +
        " onboard_control_enabled:" + std::to_string(mavlink_status_info.onboard_control_sensors_enabled) +
        " onboard_control_hp:" + std::to_string(mavlink_status_info.onboard_control_sensors_health) +
        " load:" + std::to_string(mavlink_status_info.load) +
        " drop_rate_comm:" + std::to_string(mavlink_status_info.drop_rate_comm);
        //" errors_comm:" + std::to_string(mavlink_status_info.errors_comm) +
        //" errors_count1:" + std::to_string(mavlink_status_info.errors_count1) +
        //" errors_count2:" + std::to_string(mavlink_status_info.errors_count2) +
        //" errors_count3:" + std::to_string(mavlink_status_info.errors_count3) +
        //" errors_count4:" + std::to_string(mavlink_status_info.errors_count4);
}

void MavlinkParser::MAVLINK_MSG_ID_ATTITUDE_func(mavlink_message_t msg)
{
    mavlink_attitude_t mavlink_attitude_info;
    mavlink_msg_attitude_decode(&msg, &mavlink_attitude_info);

    std::lock_guard<std::mutex> lock(telemetry_state_formater.mtx);
    telemetry_state_formater.attitude.has_value = true;

    telemetry_state_formater.attitude.last_update_point = std::chrono::steady_clock::now();
    telemetry_state_formater.attitude.text =
        "roll:" + std::to_string(mavlink_attitude_info.roll) +
        " pitch:" + std::to_string(mavlink_attitude_info.pitch) +
        " yaw:" + std::to_string(mavlink_attitude_info.yaw) +
        " rollspeed:" + std::to_string(mavlink_attitude_info.rollspeed) +
        " pitchspeed:" + std::to_string(mavlink_attitude_info.pitchspeed) +
        " yawspeed:" + std::to_string(mavlink_attitude_info.yawspeed) +
        " time_boot_ms:" + std::to_string(mavlink_attitude_info.time_boot_ms);
}

void MavlinkParser::MAVLINK_MSG_ID_GPS_RAW_INT_func(mavlink_message_t msg)
{
    mavlink_gps_raw_int_t mavlink_gps_raw;
    mavlink_msg_gps_raw_int_decode(&msg, &mavlink_gps_raw);

    std::lock_guard<std::mutex> lock(telemetry_state_formater.mtx);
    telemetry_state_formater.gps_raw.has_value = true;

    telemetry_state_formater.gps_raw.last_update_point = std::chrono::steady_clock::now();
    telemetry_state_formater.gps_raw.text =
        "time_usec:" + std::to_string(mavlink_gps_raw.time_usec) +
        " fix_type:" + std::to_string(mavlink_gps_raw.fix_type) +
        " lat:" + std::to_string(mavlink_gps_raw.lat) +
        " lon:" + std::to_string(mavlink_gps_raw.lon) +
        " alt:" + std::to_string(mavlink_gps_raw.alt) +
        " eph:" + std::to_string(mavlink_gps_raw.eph) +
        " epv:" + std::to_string(mavlink_gps_raw.epv) +
        " vel:" + std::to_string(mavlink_gps_raw.vel) +
        " cog:" + std::to_string(mavlink_gps_raw.cog) +
        " satellites_visible:" + std::to_string(mavlink_gps_raw.satellites_visible);
}

void MavlinkParser::MAVLINK_MSG_ID_GLOBAL_POSITION_INT_func(mavlink_message_t msg)
{
    mavlink_global_position_int_t mavlink_pos_info;
    mavlink_msg_global_position_int_decode(&msg, &mavlink_pos_info);

    std::lock_guard<std::mutex> lock(telemetry_state_formater.mtx);
    telemetry_state_formater.global_pos.has_value = true;

    telemetry_state_formater.global_pos.last_update_point = std::chrono::steady_clock::now();    
    telemetry_state_formater.global_pos.text =
        "time_boot_ms:" + std::to_string(mavlink_pos_info.time_boot_ms) +
        " lat:" + std::to_string(mavlink_pos_info.lat) +
        " lon:" + std::to_string(mavlink_pos_info.lon) +
        " alt:" + std::to_string(mavlink_pos_info.alt) +
        " relative_alt:" + std::to_string(mavlink_pos_info.relative_alt) +
        " vx:" + std::to_string(mavlink_pos_info.vx) +
        " vy:" + std::to_string(mavlink_pos_info.vy) +
        " vz:" + std::to_string(mavlink_pos_info.vz) +
        " hdg:" + std::to_string(mavlink_pos_info.hdg);
}

void MavlinkParser::MAVLINK_MSG_ID_RC_CHANNELS_RAW_func(mavlink_message_t msg)
{
    mavlink_rc_channels_raw_t mavlink_rc_info;
    mavlink_msg_rc_channels_raw_decode(&msg, &mavlink_rc_info);

    std::lock_guard<std::mutex> lock(telemetry_state_formater.mtx);
    telemetry_state_formater.rc_channels.has_value = true;

    telemetry_state_formater.rc_channels.last_update_point = std::chrono::steady_clock::now();
    telemetry_state_formater.rc_channels.text =
        "time_boot_ms:" + std::to_string(mavlink_rc_info.time_boot_ms) +
        " chan1_raw:" + std::to_string(mavlink_rc_info.chan1_raw) +
        " chan2_raw:" + std::to_string(mavlink_rc_info.chan2_raw) +
        " chan3_raw:" + std::to_string(mavlink_rc_info.chan3_raw) +
        " chan4_raw:" + std::to_string(mavlink_rc_info.chan4_raw) +
        " chan5_raw:" + std::to_string(mavlink_rc_info.chan5_raw) +
        " chan6_raw:" + std::to_string(mavlink_rc_info.chan6_raw) +
        " chan7_raw:" + std::to_string(mavlink_rc_info.chan7_raw) +
        " chan8_raw:" + std::to_string(mavlink_rc_info.chan8_raw) +
        " port:" + std::to_string(mavlink_rc_info.port) +
        " rssi:" + std::to_string(mavlink_rc_info.rssi);
}

void MavlinkParser::MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN_func(mavlink_message_t msg)
{
    mavlink_gps_global_origin_t gps_info;
    mavlink_msg_gps_global_origin_decode(&msg, &gps_info);

    std::lock_guard<std::mutex> lock(telemetry_state_formater.mtx);
    telemetry_state_formater.gps_global.has_value = true;

    telemetry_state_formater.gps_global.last_update_point = std::chrono::steady_clock::now();
    telemetry_state_formater.gps_global.text =
        "time_usec:" + std::to_string(gps_info.time_usec) +
        " latitude:" + std::to_string(gps_info.latitude) +
        " longitude:" + std::to_string(gps_info.longitude) +
        " altitude:" + std::to_string(gps_info.altitude);
}

void MavlinkParser::MAVLINK_MSG_ID_VFR_HUD_func(mavlink_message_t msg)
{
    mavlink_vfr_hud_t mavlink_vfr_hud;
    mavlink_msg_vfr_hud_decode(&msg, &mavlink_vfr_hud);

    std::lock_guard<std::mutex> lock(telemetry_state_formater.mtx);
    telemetry_state_formater.vfr_hud.has_value = true;

    telemetry_state_formater.vfr_hud.last_update_point = std::chrono::steady_clock::now();
    telemetry_state_formater.vfr_hud.text =
        "airspeed:" + std::to_string(mavlink_vfr_hud.airspeed) +
        " groundspeed:" + std::to_string(mavlink_vfr_hud.groundspeed) +
        " heading:" + std::to_string(mavlink_vfr_hud.heading) +
        " throttle:" + std::to_string(mavlink_vfr_hud.throttle) +
        " alt:" + std::to_string(mavlink_vfr_hud.alt) +
        " climb:" + std::to_string(mavlink_vfr_hud.climb);
}