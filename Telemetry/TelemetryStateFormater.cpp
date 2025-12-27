#include "TelemetryStateFormater.h"

void TelemetryStateFormater::reset_all()
{
    std::lock_guard<std::mutex> lock(mtx);  // Блокируем мьютекс на время сброса данных
    
    // Лямбда-функция сброса
    auto reset = [&](Entry &e)
    {
        e.text = "waiting...";  // Сбрасываем текстовое представление
        e.has_value = false;  // Сбрасываем флаг наличия значения
        e.last_update_point = std::chrono::steady_clock::time_point{};  // Сбрасываем время последнего обновления
    };

    reset(heartbeat);   // Сбрасываем все записи
    reset(sys_status);
    reset(attitude);
    reset(gps_raw);
    reset(global_pos);
    reset(vfr_hud);
    reset(rc_channels);
    reset(gps_global);
}