/*
Отвечает ТОЛЬКО за:

поиск порта

открытие/закрытие

чтение байтов
*/

#pragma once
#include <string>
#include <vector>
#include <boost/asio.hpp>

using boost::asio::serial_port;

class SerialPortManager // класс для управления последовательными портами
{
public:
    SerialPortManager(boost::asio::io_context &io_context) : io_(io_context) {}  // конструктор с инициализацией io_context
    void configure_port(serial_port &sp);                                        // настраивает параметры порта
    std::vector<std::string> list_serial_ports_windows();                        // возвращает список доступных COM портов (Windows)
    bool port_has_mavlink(boost::asio::io_context &io, const std::string &port); // проверяет, есть ли на порту mavlink
    std::string find_mavlink_port_windows();                                     // ищет порт с mavlink (Windows)

private:
    boost::asio::io_context &io_; // ссылка на io_context для асинхронных операций
};