#include "ScreenView.h"

void ScreenView::enable_ansi_on_windows()   //включает поддержку ANSI в консоли Windows
{
#ifdef _WIN32   // Проверяем, что мы на Windows
    HANDLE hOut = GetStdHandle(STD_OUTPUT_HANDLE);  // Получаем дескриптор стандартного вывода
    DWORD dwMode = 0;   // Переменная для хранения текущего режима консоли
    GetConsoleMode(hOut, &dwMode);  // Получаем текущий режим консоли
    dwMode |= ENABLE_VIRTUAL_TERMINAL_PROCESSING;   // Включаем поддержку ANSI последовательностей
    SetConsoleMode(hOut, dwMode);   // Устанавливаем новый режим консоли
    #endif
}

void ScreenView::cleanup()
{
    std::cout << "\033[?25h"; // показать курсор
    std::cout << "\033[2J";   // очистить экран
    std::cout << "\033[H";    // курсор в начало
    std::cout << std::flush;
}

void ScreenView::redraw_screen()
{
    std::cout << "\033[H"; // курсор в начало

    auto now = std::chrono::steady_clock::now();    // Текущее время для вычисления времени с последнего обновления

    auto print_entry = [&](const TelemetryStateFormater::Entry& e, const std::string& name) {   // Лямбда-функция для печати одной записи
        double seconds_since_last = -1.0;   // Инициализируем временем -1.0 (означает "никогда")
        if (e.has_value)    // Если значение есть, вычисляем время с последнего обновления
            seconds_since_last = std::chrono::duration<double>(now - e.last_update_point).count();  // в секундах

        std::cout << std::fixed << std::setprecision(6); // вывод с 6 знаками после запятой
        std::cout << name << ": " << e.text     // выводим имя и текст записи
                  << "   (last: "   // время с последнего обновления
                  << (seconds_since_last < 0 ? -1.0 : seconds_since_last)   // если -1.0, значит "никогда"
                  << "s)\033[K\n";  // очищаем до конца строки
    };

    std::lock_guard<std::mutex> lock(mavlink_parser.telemetry_state_formater.mtx);  // Блокируем мьютекс на время чтения данных

    print_entry(mavlink_parser.telemetry_state_formater.heartbeat, "HEARTBEAT");
    print_entry(mavlink_parser.telemetry_state_formater.sys_status, "SYS_STATUS");
    print_entry(mavlink_parser.telemetry_state_formater.attitude, "ATTITUDE");
    print_entry(mavlink_parser.telemetry_state_formater.gps_raw, "GPS_RAW_INT");
    print_entry(mavlink_parser.telemetry_state_formater.global_pos, "GLOBAL_POS");
    print_entry(mavlink_parser.telemetry_state_formater.vfr_hud, "VFR_HUD");
    print_entry(mavlink_parser.telemetry_state_formater.rc_channels, "RC_CHANNELS");
    print_entry(mavlink_parser.telemetry_state_formater.gps_global, "GPS_GLOBAL");
    std::cout << std::flush;    // сбрасываем буфер вывода
}

void ScreenView::signal_handler(int)    //обработчик сигналов для очистки экрана при выходе
{
    ScreenView::cleanup();    // очищаем экран перед выходом
    exit(0);    // завершаем программу
}
