/*
Хранит строки для отображения — heartbeat, gps, attitude.
Принимает mavlink_message_t, возвращает строку.
*/
#pragma once
#include <string>
#include <mutex>

class TelemetryStateFormater
{
private:
public:
    struct Entry {                              //структура для хранения состояния одного параметра
        std::string text = "waiting...";        //текстовое представление
        std::chrono::steady_clock::time_point last_update_point;  //время последнего обновления
        bool has_value = false;                     //флаг наличия значения
    };

    void reset_all();    //сбрасывает все состояния в начальное значение

    Entry heartbeat;
    Entry sys_status;
    Entry attitude;
    Entry gps_raw;
    Entry global_pos;
    Entry vfr_hud;
    Entry rc_channels;
    Entry gps_global;

    std::mutex mtx;
    //void update(Entry& e, const std::string& text, double now);     //обновляет состояние 
};