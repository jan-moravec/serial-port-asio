#include <iostream>
#include <array>
#include <chrono>

#include "library/serial.h"
#include "library/platform.h"

/// Test speed of serial device connected to loop
/// It generates messages, sends them, recieves them and checks the values.
void testSpeed(Serial &serial, unsigned bps)
{
    if (serial.setBaudRate(bps) != 0) {
        std::cout << "Error setting baud rate " << bps << std::endl;
        return;
    }
    serial.setFlowControl(boost::asio::serial_port_base::flow_control::type::none);
    serial.setParity(boost::asio::serial_port_base::parity::type::none);
    serial.setStopBits(boost::asio::serial_port_base::stop_bits::type::one);
    serial.setCharacterSize(8);

    const unsigned buffer_size = 256;
    std::array<uint8_t, buffer_size> buffer_read{};
    std::array<uint8_t, buffer_size> buffer_write{};

    // Read all pending data
    for (unsigned i = 0; i < 10; ++i) {
        serial.readSome(buffer_read.data(), buffer_size, std::chrono::milliseconds(10));
    }

    const unsigned testing_cycles = bps / 8 * 2 / buffer_size;
    unsigned cycles = 0;
    std::chrono::time_point<std::chrono::steady_clock> start = std::chrono::steady_clock::now();
    for (unsigned i = 0; i < testing_cycles; ++i) {
        // Generate data
        for (unsigned j = 0; j < buffer_size; j++) {
            buffer_write[j] = i + j;
        }

        int length;
        length = serial.write(buffer_write.data(), buffer_size, std::chrono::seconds(1));
        if (length != buffer_size) {
            std::cout << "Error write " << bps  << ": testing_cycle " << i << std::endl;
            return;
        }

        length = serial.readExactly(buffer_read.data(), buffer_size, std::chrono::seconds(1));
        if (length != buffer_size) {
            std::cout << "Error read " << bps  << ": testing_cycle " << i << std::endl;
            return;
        }

        // Check data
        for (unsigned j = 0; j < buffer_size; ++j) {
            if (buffer_write[j] != buffer_read[j]) {
                std::cout << "Error check " << bps << ": testing_cycle " << i << ", buffer_write " << (int)buffer_write[j]
                          << ", buffer_read " << (int)buffer_read[j] << std::endl;
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

/// Example of a child for Serial class with asynchronous read
class SerialTest: public Serial
{
public:
    SerialTest(const std::string &port): Serial(port) {
        if (isOpened()) {
            setBaudRate(115200);
            setFlowControl(boost::asio::serial_port_base::flow_control::type::none);
            setParity(boost::asio::serial_port_base::parity::type::none);
            setStopBits(boost::asio::serial_port_base::stop_bits::type::one);
            setCharacterSize(8);

            readAsynchronousStart();
        }
    }
};

int main()
{
    std::vector<PortInfo> ports = GetSerialPorts();

    std::cout << "Found " << ports.size() << " serial port devices." << std::endl;

    for (const PortInfo &port: ports) {
        std::cout << "Port " << port.port << std::endl;
        std::cout << "      - " << port.device << std::endl;
        std::cout << "      - " << port.description << std::endl;
        std::cout << "      - " << port.hardware_id << std::endl;

        Serial serial(port.device);

        if (serial) {
            std::vector<unsigned> speeds = {4800, 9600, 19200, 57600, 115200, 230400, 460800, 576000, 921600, 1152000}; // , 2000000, 4000000

            for (unsigned &speed: speeds) {
                testSpeed(serial, speed);
            }
        }
    }

    if (ports.size() > 0) {
        std::cout << "Testing functions " << ports.at(0).device << std::endl;
        Serial serial(ports.at(0).device);
        if (serial) {
            serial.setBaudRate(115200);
            serial.setFlowControl(boost::asio::serial_port_base::flow_control::type::none);
            serial.setParity(boost::asio::serial_port_base::parity::type::none);
            serial.setStopBits(boost::asio::serial_port_base::stop_bits::type::one);
            serial.setCharacterSize(8);

            std::string message;
            message.resize(128);

            serial.write("Testing 123...");
            serial.readSome(message);
            std::cout << message << std::endl;

            message.clear();
            serial.write("Testing 456...\n");
            serial.readUntil(message, "\n");
            std::cout << message;
        }
    }

    if (ports.size() > 0) {
        std::cout << "Testing asynchronous read " << ports.at(0).device << std::endl;
        SerialTest test(ports.at(0).device);

        if (test) {
            for (int i = 0; i < 4; ++i) {
                test.write("Testing " + std::to_string(i));
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
        }
    }

    return 0;
}
