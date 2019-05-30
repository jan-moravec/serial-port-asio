#include <iostream>
#include <array>
#include <chrono>

#include "library/serial.h"
#include "library/platform.h"

void testSpeed(Serial &serial, unsigned bps)
{
    serial.setBaudRate(bps);

    const unsigned buffer_size = 256;
    std::array<uint8_t, buffer_size> buffer_read;
    std::array<uint8_t, buffer_size> buffer_write;

    // Read all pending data
    for (unsigned i = 0; i < 10; ++i) {
        serial.readSome(buffer_read.data(), buffer_size, std::chrono::milliseconds(1));
    }

    const unsigned testing_cycles = bps / 8 * 2 / buffer_size;
    unsigned cycles = 0;
    std::chrono::time_point<std::chrono::steady_clock> start = std::chrono::steady_clock::now();
    for (unsigned i = 0; i < testing_cycles; ++i) {
        for (unsigned j = 0; j < buffer_size; j++) {
            buffer_write[j] = i + j;
        }

        int length;
        length = serial.write(buffer_write.data(), buffer_size, std::chrono::seconds(1));
        if (length != buffer_size) {
            std::cout << "Error write " << bps << std::endl;
            return;
        }

        length = serial.readExactly(buffer_read.data(), buffer_size, std::chrono::seconds(1));
        if (length != buffer_size) {
            std::cout << "Error read " << bps << std::endl;
            return;
        }

        for (unsigned j = 0; j < buffer_size; ++j) {
            if (buffer_write[j] != buffer_read[j]) {
                std::cout << "Error check " << bps << std::endl;
                return;
            }
        }

        cycles++;
    }
    std::chrono::time_point<std::chrono::steady_clock> stop = std::chrono::steady_clock::now();

    unsigned took = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start).count();
    double speed = (cycles * buffer_size) * 1000.0 / took;
    std::cout << "Bitrate: " << bps << " bps, Speed: " << speed << " B/s = " << speed / 1000.0 << " KB/s = " <<  speed / 1000000.0 << " MB/s" << std::endl;
}

int main()
{
    std::vector<PortInfo> ports = listPorts();
    std::cout << ports.size() << std::endl;

    for (const PortInfo &port: ports) {
        std::cout << port.port << " -- " << port.description << " -- " << port.hardware_id << std::endl;
    }
    /*Serial serial("/dev/ttyUSB0");

    if (serial) {
        std::vector<unsigned> speeds = {4800, 9600, 19200, 57600, 115200, 230400, 460800, 576000, 921600, 1152000, 2000000, 4000000};

        for (unsigned &speed: speeds) {
            testSpeed(serial, speed);
        }
    }*/

    return 0;
}
