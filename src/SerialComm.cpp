#include <stdio.h>
#include <string.h>

#include "SerialComm.hpp"

// File controls like O_RDWR
#include <fcntl.h>

// Error integer and strerror function
#include <errno.h>

// POSIX terminal control definitions
#include <termios.h>

// write(), read(), close()
#include <unistd.h>

namespace SerialComm
{

    SerialPort::SerialPort() : fd(-1) {}

    SerialPort::~SerialPort()
    {
        if (fd != -1)
        {
            close();
        }
    }

    SerialError SerialPort::open(const std::string &device_name)
    {
        fd = ::open(device_name.c_str(), O_RDWR);
        if (fd < 0)
        {
            return SerialError::ERROR_OPEN_DEVICE;
        }
        return SerialError::SUCCESS;
    }

    SerialError SerialPort::configure(BAUDRATE baud_rate)
    {
        if (tcgetattr(fd, &tty) != 0)
        {
            return SerialError::ERROR_CONFIGURE_DEVICE;
        }
        cfsetspeed(&tty, speed_t(baud_rate));
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;
        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        if (tcsetattr(fd, TCSANOW, &tty) != 0)
        {
            return SerialError::ERROR_CONFIGURE_DEVICE;
        }
        return SerialError::SUCCESS;
    }

    SerialError SerialPort::sendMessage(const std::string &message)
    {
        int bytes_written = write(fd, message.c_str(), message.size());
        return (bytes_written == static_cast<int>(message.size())) ? SerialError::SUCCESS : SerialError::ERROR_CONFIGURE_DEVICE;
    }

    int SerialPort::receiveMessage(char *buffer, size_t buffer_size)
    {
        int bytes_read = read(fd, buffer, buffer_size - 1);
        if (bytes_read > 0)
        {
            buffer[bytes_read] = '\0';
        }
        return bytes_read;
    }

    void SerialPort::close()
    {
        ::close(fd);
        fd = -1;
    }

} // namespace SerialComm

// int main () {
//     int device = open("/dev/cu.usbmodemHDC2XXX1", O_RDWR);
    
//     if (device < 0)
//     {
//         printf("Error %i from open: %s\n", errno, strerror(errno));
//     }

//     // Need access to termios struct to configure serial port
//     struct termios tty;

//     if (tcgetattr(device, &tty) != 0) 
//     {
//         printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
//     }

//     /*
//     HDC246o Serial  communication parameters
//     Baud rate: 115200
//     Data bits: 8
//     Parity: None
//     Stop bits: 1
//     Start bits: 1
    
//     */
//     tty.c_cflag &= ~CSIZE;           // Clear data bit size
//     tty.c_cflag |= CS8;              // Set 8 data bits
//     tty.c_cflag &= ~PARENB;          // No parity
//     tty.c_cflag &= ~CSTOPB;          // 1 stop bit
//     tty.c_cflag |= (CLOCAL | CREAD); // Enable receiver, ignore modem control lines

//     // Input flags
//     tty.c_iflag |= (IGNPAR | IGNBRK); // Ignore parity errors and break condition
//     tty.c_iflag &= ~(ICRNL | INLCR);  // Disable CR to NL conversion

//     // Output flags
//     tty.c_oflag &= ~OPOST; // Disable output processing

//     // Local flags
//     tty.c_lflag &= ~(ICANON | ISIG | ECHO); // Set raw input mode

//     // Control characters (non-blocking read)
//     tty.c_cc[VMIN] = 0;
//     tty.c_cc[VTIME] = 1;

//     cfsetspeed(&tty, B115200);

//     // Save tty termios struct with tcsetattr()
//     if (tcsetattr(device, TCSANOW, &tty) != 0) 
//     {
//         printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
//     }

//     // Sending the message "?C\r"
//     char message[] = "?C\r";
//     int bytes_written = write(device, message, sizeof(message) - 1);
//     if (bytes_written < 0)
//     {
//         printf("Error %i from write: %s\n", errno, strerror(errno));
//     }
//     else
//     {
//         printf("Sent %d bytes: %s\n", bytes_written, message);
//     }

//     // Reading the response
//     char read_buffer[256];
//     memset(read_buffer, 0, sizeof(read_buffer));
//     int bytes_read = read(device, read_buffer, sizeof(read_buffer) - 1);
//     if (bytes_read < 0)
//     {
//         printf("Error %i from read: %s\n", errno, strerror(errno));
//     }
//     else if (bytes_read == 0)
//     {
//         printf("No response received.\n");
//     }
//     else
//     {
//         printf("Received %d bytes: %s\n", bytes_read, read_buffer);
//     }

//     // Close the serial device
//     close(device);

//     return 0;
// }
// Check for errors
