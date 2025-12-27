//Отвечает за вывод на консоль.
#pragma once

#include "../mavlink/MavlinkParser.h"
#include "../telemetry/TelemetryStateFormater.h"
#include <csignal>
#include <iostream>
#include <iomanip>  // для std::fixed и std::setprecision
#include <chrono>

#ifdef _WIN32
    #include <windows.h>
#endif

class ScreenView
{
public:
    ScreenView(MavlinkParser& parser) : mavlink_parser(parser){
        // Регистрация сигналов при создании объекта
        std::signal(SIGINT, ScreenView::signal_handler);    // ловим Ctrl+C
        std::signal(SIGTERM, ScreenView::signal_handler);   // ловим сигнал завершения
    }

    void enable_ansi_on_windows();  //включает поддержку ANSI в консоли Windows
    static void cleanup();     // очистка при выходе
    void redraw_screen();   //перерисовка экрана/обновление данных
    static void signal_handler(int);    //обработчик сигналов для очистки экрана при выходе
    
private:
    MavlinkParser& mavlink_parser;   //парсер mavlink сообщений
};