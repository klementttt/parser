#include "SerialPortManager.h"

void SerialPortManager::configure_port(boost::asio::serial_port& sp)        //настраивает параметры порта
{
    using boost::asio::serial_port_base;

    sp.set_option(serial_port_base::baud_rate(57600));  // Устанавливаем скорость порта
    sp.set_option(serial_port_base::character_size(8));     // Размер символа - 8 бит
    sp.set_option(serial_port_base::parity(serial_port_base::parity::none));    // Без проверки четности
    sp.set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one));   // 1 стоп-бит
    sp.set_option(serial_port_base::flow_control(serial_port_base::flow_control::none));    // Без управления потоком

    HANDLE h = sp.native_handle();  // Получаем нативный дескриптор порта
    COMMTIMEOUTS timeouts = { 0 };  // Инициализируем структуру таймаутов
    timeouts.ReadIntervalTimeout = 50;  // Устанавливаем интервальный таймаут на максимум
    timeouts.ReadTotalTimeoutMultiplier = 0;    // Общий таймаут чтения - 0
    timeouts.ReadTotalTimeoutConstant = 200;    // Общий таймаут умножителя - 0
    SetCommTimeouts(h, &timeouts);  // Применяем таймауты к порту
}

std::vector<std::string> SerialPortManager::list_serial_ports_windows()         //возвращает список доступных COM портов (Windows)
{
    std::vector<std::string> ports;     // Вектор для хранения имен портов
    char portName[16];      // Буфер для имени порта

    for (int i = 1; i < 256; i++)   // Перебираем возможные номера COM портов
    {
        sprintf(portName, "\\\\.\\COM%d", i);       // Формируем имя порта

        HANDLE h = CreateFileA(portName,        // Пытаемся открыть порт
            GENERIC_READ | GENERIC_WRITE,       // Режим чтения и записи
            0,      // Нет совместного доступа
            NULL,   // Нет специальных атрибутов безопасности
            OPEN_EXISTING,      // Открываем существующий порт
            0,      // Стандартные атрибуты
            NULL);      // Нет шаблона

        if (h != INVALID_HANDLE_VALUE)  // Если порт успешно открыт
        {
            ports.push_back(std::string("COM") + std::to_string(i));        // Добавляем имя порта в список
            CloseHandle(h);     // Закрываем дескриптор порта
        }
    }

    return ports;       // Возвращаем список найденных портов
}

bool SerialPortManager::port_has_mavlink(boost::asio::io_context& io, const std::string& port)   //проверяет, есть ли на порту mavlink
{
    boost::asio::serial_port sp(io);    // Создаем объект serial_port
    boost::system::error_code ec;       // Объект для хранения ошибок

    sp.open(port, ec);      // Пытаемся открыть указанный порт
    if (ec) return false;       // Если ошибка при открытии, возвращаем false

    configure_port(sp); // Настраиваем параметры порта

    uint8_t buf[1];     // Буфер для чтения данных

    auto start = std::chrono::steady_clock::now();      // Запоминаем время начала проверки

    while (true)     // Бесконечный цикл для чтения данных
    {
        size_t n = sp.read_some(boost::asio::buffer(buf), ec);      // Читаем данные из порта

        if (n > 0)      // Если прочитаны данные и найденны байты
        {
            for (size_t i = 0; i < n; i++)      // Перебираем прочитанные байты
            {
                if (buf[i] == 0xFE || buf[i] == 0xFD)       // Проверяем на наличие MAVLink стартовых байтов
                {
                    sp.close();         // Закрываем порт при обнаружении MAVLink стартовыйх байтов
                    return true;        // Возвращаем true
                }
            }
        }

        // Таймаут 300 мс
        if (std::chrono::steady_clock::now() - start > std::chrono::milliseconds(300))      // Проверяем, не истек ли таймаут
            break;      // Прерываем цикл по истечении таймаута
    }

    sp.close();     // Закрываем порт после проверки
    return false;   // Возвращаем false, если MAVLink не найден
}

std::string SerialPortManager::find_mavlink_port_windows()  //ищет порт с mavlink (Windows)
{
    boost::asio::io_context io;   // Создаем io_context для работы с асинхронными операциями

    std::vector<std::string> ports = list_serial_ports_windows();   // Получаем список доступных COM портов

    for (auto& p : ports)   // Перебираем каждый порт в списке
    {
        if (port_has_mavlink(io, p)){   // Проверяем, есть ли на порту MAVLink
            return p;     // нашли MAVLink порт
        }
    }

    return ""; // не найден
}