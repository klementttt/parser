/*

Отвечает ТОЛЬКО за:

принятие байтов

сборку Mavlink сообщений

декодирование сообщений

за время (секундомер)
*/


/*
#pragma once
#include "../telemetry/TelemetryModel.h"
#include "../udp/UdpSender.h"
#include "../c_library_v2-master/all/mavlink.h"
*/

#pragma once
#include "../telemetry/TelemetryStateFormater.h"
#include "../c_library_v2-master/all/mavlink.h"
#include <chrono>
#include <string>
#include <mutex>


class MavlinkParser
{
private:
    using Clock = std::chrono::steady_clock;    //секундомер
    std::chrono::time_point<Clock> start_time = Clock::now();  //время старта секундомера

public:    
    void HEARTBEAT_func(mavlink_message_t msg);
    void SYS_STATUS_func(mavlink_message_t msg);
    void MAVLINK_MSG_ID_ATTITUDE_func(mavlink_message_t msg);
    void MAVLINK_MSG_ID_GPS_RAW_INT_func(mavlink_message_t msg);
    void MAVLINK_MSG_ID_GLOBAL_POSITION_INT_func(mavlink_message_t msg);
    void MAVLINK_MSG_ID_RC_CHANNELS_RAW_func(mavlink_message_t msg);
    void MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN_func(mavlink_message_t msg);
    void MAVLINK_MSG_ID_VFR_HUD_func(mavlink_message_t msg);

    //MavlinkParser() = default;
    
    TelemetryStateFormater telemetry_state_formater;

    mavlink_message_t msg;  //текущее сообщение
    mavlink_status_t status;   // статус парсера mavlink
    //MavlinkParser(mavlink_message_t message) : msg(message) {}

    double now_sec()        //возвращает время в секундах с момента старта 
    {
        return std::chrono::duration<double>(Clock::now() - start_time).count();
    }

};