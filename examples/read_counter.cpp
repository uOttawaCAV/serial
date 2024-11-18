#include "SerialComm.hpp"
#include <iostream>
#include <chrono>
#include <thread>

int main()
{
    SerialComm::SerialPort device;
    if (device.open("/dev/cu.usbmodemHDC2XXX1") != SerialComm::SerialError::SUCCESS)
    {
        std::cerr << "Failed to open serial device\n";
        return 1;
    }

    if (device.configure(SerialComm::BAUDRATE::BAUD_115200) != SerialComm::SerialError::SUCCESS)
    {
        std::cerr << "Failed to configure serial device\n";
        device.close();
        return 1;
    }


    while (true) {
        //device.sendMessage("?C\r");

        // char buffer[256];
        // int bytes_received = device.receiveMessage(buffer, sizeof(buffer));
        // if (bytes_received > 0)
        // {
        //     std::cout << "Received: " << buffer << '\n';
        // }
        // else
        // {
        //     std::cerr << "No response received\n";
        // }

        char buffer[256];
        int bytes_received = device.sendAndReceiveMessage("?C\r", buffer, sizeof(buffer), 1000);
        if (bytes_received > 0)
        {
            std::cout << "Received: " << buffer << '\n';
        }
        else
        {
            std::cerr << "No response received\n";
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    // device.sendMessage("?C\r");

    // char buffer[256];
    // int bytes_received = device.receiveMessage(buffer, sizeof(buffer));
    // if (bytes_received > 0)
    // {
    //     std::cout << "Received: " << buffer << '\n';
    // }
    // else
    // {
    //     std::cerr << "No response received\n";
    // }

    device.close();
    return 0;
}
