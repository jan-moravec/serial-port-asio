#ifndef SERIAL_H
#define SERIAL_H

#include <memory>
#include <chrono>
#include <boost/asio.hpp>

class Serial
{
public:
    /// Create closed Serial instance
    Serial();
    /// Create instance of Serial and open port device
    Serial(const std::string &port);
    virtual ~Serial();

    Serial(const Serial &) = delete;
    Serial &operator=(const Serial &) = delete;

    /// Open port device.
    /// Use full name or path ("/dev/ttyUSB0" or "COM3" for example).
    void open(const std::string &port);
    /// Close the device. Need to close device before opening a new one.
    /// The destructor will close the device automatically.
    void close();

public:
    /// Serial port settings functions
    int setBaudRate(unsigned rate);
    int getBaudRate(unsigned &rate);
    int setFlowControl(boost::asio::serial_port_base::flow_control::type flow);
    int getFlowControl(boost::asio::serial_port_base::flow_control::type &flow);
    int setParity(boost::asio::serial_port_base::parity::type parity);
    int getParity(boost::asio::serial_port_base::parity::type &parity);
    int setStopBits(boost::asio::serial_port_base::stop_bits::type bits);
    int getStopBits(boost::asio::serial_port_base::stop_bits::type &bits);
    int setCharacterSize(unsigned size);
    int getCharacterSize(unsigned &size);

public:
    /// Following are all synchronous operations.
    /// There is always a variant with timeout and a variant without timeout (blocking).
    /// The buffer can be represented by uint8_t array or std::string. The string must have some size reserved.

    /// Transfer buffer via serial port.
    /// Returns the number of bytes written or -1 for error.
    int write(const uint8_t *buffer, std::size_t size);
    int write(const std::string &message);
    int write(const uint8_t *buffer, std::size_t size, const std::chrono::steady_clock::duration &timeout);
    int write(const std::string &message, const std::chrono::steady_clock::duration &timeout);

    /// Read some number to buffer. Maximum is size or string size.
    /// Returns the number of bytes written or -1 for error.
    int readSome(uint8_t *buffer, std::size_t size);
    int readSome(std::string &message);
    int readSome(uint8_t *buffer, std::size_t size, const std::chrono::steady_clock::duration &timeout);
    int readSome(std::string &message, const std::chrono::steady_clock::duration &timeout);

    /// Read exactly the size of data.
    /// Returns the number of bytes written or -1 for error.
    int readExactly(uint8_t *buffer, std::size_t size);
    int readExactly(std::string &message);
    int readExactly(uint8_t *buffer, std::size_t size, const std::chrono::steady_clock::duration &timeout);
    int readExactly(std::string &message, const std::chrono::steady_clock::duration &timeout);

    /// Read until delim recieved. Can return bytes after the delim.
    /// readUntil is the only function enlarging the string if needed (dynamically).
    /// Returns the number of bytes written or -1 for error.
    int readUntil(std::string &buffer, const std::string &delim);
    int readUntil(std::string &buffer, const std::string &delim, const std::chrono::steady_clock::duration &timeout);

    /// Check if device is opened.
    operator bool() const { return opened; }
    bool isOpened() const { return opened; }

protected:
    /// Asynchronous read loop.
    /// Can be used with write operation without the timeout. Do not use with the timeout functions.
    /// The timeout function will cause the io_context to stop.

    /// Start the asynchronous read loop.
    /// To stop the loop, close the serial instance.
    void readAsynchronousStart(unsigned buffer_size = 128);

    /// Override this function for data processing.
    virtual void readAsynchronousProcess(uint8_t *buffer, size_t size);

private:
    /// Buffer for asynchronous read.
    unsigned buffer_read_size;
    std::unique_ptr<uint8_t[]> buffer_read;

    /// Read new data to buffer and call the process function.
    void readAsynchronous();

private:
    std::string port;

    bool opened = false;
    bool debug = true;

    std::unique_ptr<boost::asio::io_context> io_context;
    std::unique_ptr<boost::asio::serial_port> serial_port;

    bool runFor(const std::chrono::steady_clock::duration &timeout);
    bool checkError(const boost::system::error_code &ec, bool timedout, const std::string &text);
    bool checkError(const boost::system::error_code &ec, const std::string &text);
};

#endif // SERIAL_H
