#include "PortScanner/SerialPortManager.h"
#include "Telemetry/TelemetryStateFormater.h"
#include "mavlink/MavlinkParser.h"
#include "ScreenView/ScreenView.h"

#include <iostream>
#include <boost/asio.hpp>
#include "c_library_v2-master/all/mavlink.h"
#include <thread>
#include <chrono>

using boost::asio::io_context;
using boost::asio::serial_port;

enum class LinkState    //состояние связи с mavlink
{
    SEARCHING_PORT,
    CONNECTED,
};

int main()
{

    boost::asio::io_context io; // Создаем io_context для работы с асинхронными операциями
    boost::asio::ip::udp::endpoint send_to(boost::asio::ip::make_address("172.18.167.206"), 9000);  // Адрес и порт назначения
    boost::asio::ip::udp::socket sock(io, boost::asio::ip::udp::v4());  // Создаем UDP сокет
    boost::asio::serial_port serial(io);    // Создаем объект serial_port
    
    MavlinkParser mavlink_parser;   //парсер mavlink сообщений
    SerialPortManager port_manager {io};    //менеджер COM портов
    ScreenView screen_view {mavlink_parser};    //отвечает за вывод на консоль

    LinkState link_state = LinkState::SEARCHING_PORT;   //текущее состояние связи c mavlink

    screen_view.enable_ansi_on_windows();   //включаем поддержку ANSI в консоли Windows
    std::cout << "\033[?25l\033[2J";   // скрыть курсор и очистить экран
    
    std::string current_port;   //текущий COM порт с mavlink
    std::vector<uint8_t> buffer(1); // Буфер для чтения данных из serial порта

    while (true)    //главный цикл программы
    {
        switch (link_state) //обработка текущего состояния связи
        {
            // -------------------------
            //    SEARCHING PORT
            // -------------------------
            case LinkState::SEARCHING_PORT: //поиск порта с mavlink
            {
                // очистка данных                
                mavlink_parser.telemetry_state_formater.reset_all(); //сброс состояния телеметрии
                screen_view.redraw_screen();    //обновление экрана
                current_port = port_manager.find_mavlink_port_windows();    //ищет порт с mavlink (Windows)

                mavlink_parser.status = mavlink_status_t{}; //сброс статуса парсера
                mavlink_parser.msg = mavlink_message_t{};   //сброс текущего сообщения

                if (!current_port.empty())  //если нашли порт с mavlink
                {
                    std::cout << "MAVLink port found: " << current_port << "\n";    //выводим информацию в консоль

                    boost::system::error_code ec;   //код ошибки
                    serial.open(current_port, ec);  //открываем порт
                    if (!ec)    //если ошибок нет
                    {
                        port_manager.configure_port(serial);    //настройка параметров порта
                        link_state = LinkState::CONNECTED;  //переходим в состояние CONNECTED
                    }
                }

                break;
            }

            // -------------------------
            //      CONNECTED
            // -------------------------
            case LinkState::CONNECTED:
            {
                boost::system::error_code ec;   //код ошибки
                size_t n = serial.read_some(boost::asio::buffer(buffer), ec);      // Читаем данные из порта

                if (ec) //если произошла ошибка при чтении
                {
                    std::cout << "Connection lost.\n";  //выводим информацию в консоль
                    serial.close();   //закрываем порт
                    current_port.clear();   //очищаем текущий порт
                    link_state = LinkState::SEARCHING_PORT;  //переходим в состояние SEARCHING_PORT
                    break;
                }

                if (n > 0)  //если прочитаны данные
                {
                    sock.send_to(boost::asio::buffer(&buffer[0], 1), send_to);  //пересылаем байт по UDP
                    if (mavlink_parse_char(MAVLINK_COMM_0, buffer[0], &mavlink_parser.msg, &mavlink_parser.status)) //парсим байт
                    {
                        switch (mavlink_parser.msg.msgid)   //обрабатываем сообщение по его ID
                        {
                        case MAVLINK_MSG_ID_HEARTBEAT:  //если сообщение HEARTBEAT
                            mavlink_parser.HEARTBEAT_func(mavlink_parser.msg);  //вызываем соответствующую функцию
                            screen_view.redraw_screen();    //обновление экрана
                            break;
                        case MAVLINK_MSG_ID_SYS_STATUS:
                            mavlink_parser.SYS_STATUS_func(mavlink_parser.msg);
                            screen_view.redraw_screen();    //обновление экрана
                            break;
                        case MAVLINK_MSG_ID_ATTITUDE:
                            mavlink_parser.MAVLINK_MSG_ID_ATTITUDE_func(mavlink_parser.msg);
                            screen_view.redraw_screen();    //обновление экрана
                            break;
                        case MAVLINK_MSG_ID_GPS_RAW_INT:
                            mavlink_parser.MAVLINK_MSG_ID_GPS_RAW_INT_func(mavlink_parser.msg);
                            screen_view.redraw_screen();    //обновление экрана
                            break;  
                        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
                            mavlink_parser.MAVLINK_MSG_ID_GLOBAL_POSITION_INT_func(mavlink_parser.msg);
                            break;
                        case MAVLINK_MSG_ID_RC_CHANNELS_RAW:
                            mavlink_parser.MAVLINK_MSG_ID_RC_CHANNELS_RAW_func(mavlink_parser.msg);
                            screen_view.redraw_screen();    //обновление экрана
                            break;
                        case MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN:
                            mavlink_parser.MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN_func(mavlink_parser.msg);
                            screen_view.redraw_screen();    //обновление экрана                            
                            break;
                        case MAVLINK_MSG_ID_VFR_HUD:
                            mavlink_parser.MAVLINK_MSG_ID_VFR_HUD_func(mavlink_parser.msg);
                            screen_view.redraw_screen();    //обновление экрана
                            break;
                        }
                    }
                    //screen_view.redraw_screen();    //обновление экрана
                }
                break;
            }
        }
    }
    screen_view.cleanup();  // очистка при выходе
}
