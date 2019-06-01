#ifndef SERIAL_H
#define SERIAL_H

#include <memory>
#include <chrono>
#include <boost/asio.hpp>

class Serial
{
public:
    Serial();
    Serial(const std::string &port);
    virtual ~Serial();

    Serial(const Serial&) = delete;
    Serial& operator=(const Serial&) = delete;

    void open(const std::string &port);
    void close();

    /// Settings
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

    /// Synchronous operations
    int write(const uint8_t *buffer, std::size_t size);
    int write(const std::string &message);
    int write(const uint8_t *buffer, std::size_t size, const std::chrono::steady_clock::duration &timeout);
    int write(const std::string &message, const std::chrono::steady_clock::duration &timeout);

    int readSome(uint8_t *buffer, std::size_t size);
    int readSome(std::string &message);
    int readSome(uint8_t *buffer, std::size_t size, const std::chrono::steady_clock::duration &timeout);
    int readSome(std::string &message, const std::chrono::steady_clock::duration &timeout);
    int readExactly(uint8_t *buffer, std::size_t size);
    int readExactly(std::string &message);
    int readExactly(uint8_t *buffer, std::size_t size, const std::chrono::steady_clock::duration &timeout);
    int readExactly(std::string &message, const std::chrono::steady_clock::duration &timeout);
    int readUntil(std::string &buffer, const std::string &delim);
    int readUntil(std::string &buffer, const std::string &delim, const std::chrono::steady_clock::duration &timeout);

    operator bool() const { return opened; }
    bool isOpened() const { return opened; }

protected:
    /// Asynchronous read thread
    /// Can be used with write operation without the timeout. Do not use with the timeout functions.
    static const unsigned buffer_read_size = 512;
    std::unique_ptr<uint8_t[]> buffer_read;
    void readAsyncStart();
    void readAsync();
    void readAsyncRecieve(const boost::system::error_code& ec, size_t size);
    /// Override for data processing.
    virtual void readAsyncProcess(size_t size);

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
