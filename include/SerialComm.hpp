#ifndef SerialPort_HPP // Include guard
#define SerialPort_HPP

#include <string>
#include <termios.h>

namespace SerialComm
{

    // Error codes
    enum class SerialError
    {
        SUCCESS = 0,
        ERROR_OPEN_DEVICE = -1,
        ERROR_CONFIGURE_DEVICE = -2
    };

    enum class BAUDRATE
    {
        BAUD_115200 = 115200
    };

    class SerialPort
    {
    private:
        int fd;             // File descriptor for the serial device
        struct termios tty; // Termios structure for serial configuration

    public:
        // Constructor and Destructor
        SerialPort();
        ~SerialPort();

        // Methods
        SerialError open(const std::string &device_name);
        SerialError configure(BAUDRATE baud_rate);
        SerialError sendMessage(const std::string &message);
        int receiveMessage(char *buffer, size_t buffer_size);
        void close();
    };

} // namespace SerialComm

#endif // SerialPort_HPP
