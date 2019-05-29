#include "serial.h"
#include <iostream>

Serial::Serial()
{

}

Serial::Serial(const std::string &port)
{
    open(port);
}

Serial::~Serial()
{
    close();
}

void Serial::open(const std::string &port)
{
    if (opened) {
        return;
    }

    io_context = std::make_unique<boost::asio::io_context>();
    serial_port = std::make_unique<boost::asio::serial_port>(*io_context);

    boost::system::error_code ec;
    serial_port->open(port, ec);

    if (checkError(ec, __PRETTY_FUNCTION__)) {
        return;
    }

    serial_port->set_option(boost::asio::serial_port_base::baud_rate(9600));
    serial_port->set_option(boost::asio::serial_port_base::character_size(8));
    serial_port->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
    serial_port->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
    serial_port->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));

    this->port = port;
    opened = true;
}

void Serial::close()
{
    if (opened) {
        serial_port->cancel();
        serial_port->close();
        io_context->stop();

        opened = false;
    }
}

int Serial::setBaudRate(unsigned rate)
{
    boost::system::error_code ec;
    serial_port->set_option(boost::asio::serial_port_base::baud_rate(rate), ec);

    if (checkError(ec, __PRETTY_FUNCTION__)) {
        return -1;
    }

    return 0;
}

int Serial::getBaudRate(unsigned &rate)
{
    boost::system::error_code ec;
    boost::asio::serial_port_base::baud_rate baudrate;
    serial_port->get_option(baudrate, ec);

    if (checkError(ec, __PRETTY_FUNCTION__)) {
        return -1;
    }

    rate = baudrate.value();
    return 0;
}

int Serial::setFlowControl(boost::asio::serial_port_base::flow_control::type flow)
{
    boost::system::error_code ec;
    serial_port->set_option(boost::asio::serial_port_base::flow_control(flow), ec);

    if (checkError(ec, __PRETTY_FUNCTION__)) {
        return -1;
    }

    return 0;
}

int Serial::getFlowControl(boost::asio::serial_port_base::flow_control::type &flow)
{
    boost::system::error_code ec;
    boost::asio::serial_port_base::flow_control flowcontrol;
    serial_port->get_option(flowcontrol, ec);

    if (checkError(ec, __PRETTY_FUNCTION__)) {
        return -1;
    }

    flow = flowcontrol.value();
    return 0;
}

int Serial::setParity(boost::asio::serial_port_base::parity::type parity)
{
    boost::system::error_code ec;
    serial_port->set_option(boost::asio::serial_port_base::parity(parity), ec);

    if (checkError(ec, __PRETTY_FUNCTION__)) {
        return -1;
    }

    return 0;
}

int Serial::getParity(boost::asio::serial_port_base::parity::type &parity)
{
    boost::system::error_code ec;
    boost::asio::serial_port_base::parity par;
    serial_port->get_option(par, ec);

    if (checkError(ec, __PRETTY_FUNCTION__)) {
        return -1;
    }

    parity = par.value();
    return 0;
}

int Serial::setStopBits(boost::asio::serial_port_base::stop_bits::type bits)
{
    boost::system::error_code ec;
    serial_port->set_option(boost::asio::serial_port_base::stop_bits(bits), ec);

    if (checkError(ec, __PRETTY_FUNCTION__)) {
        return -1;
    }

    return 0;
}

int Serial::getStopBits(boost::asio::serial_port_base::stop_bits::type &bits)
{
    boost::system::error_code ec;
    boost::asio::serial_port_base::stop_bits stopbits;
    serial_port->get_option(stopbits, ec);

    if (checkError(ec, __PRETTY_FUNCTION__)) {
        return -1;
    }

    bits = stopbits.value();
    return 0;
}

int Serial::setCharacterSize(unsigned size)
{
    boost::system::error_code ec;
    serial_port->set_option(boost::asio::serial_port_base::character_size(size), ec);

    if (checkError(ec, __PRETTY_FUNCTION__)) {
        return -1;
    }

    return 0;
}

int Serial::getCharacterSize(unsigned &size)
{
    boost::system::error_code ec;
    boost::asio::serial_port_base::character_size charsize;
    serial_port->get_option(charsize, ec);

    if (checkError(ec, __PRETTY_FUNCTION__)) {
        return -1;
    }

    size = charsize.value();
    return 0;
}

int Serial::write(const uint8_t *buffer, std::size_t size)
{
    boost::system::error_code ec;
    size_t length = boost::asio::write(*serial_port, boost::asio::buffer(buffer, size), ec);

    if (checkError(ec, __PRETTY_FUNCTION__)) {
        return -1;
    }

    return length;
}

int Serial::write(const uint8_t *buffer, std::size_t size, const std::chrono::steady_clock::duration &timeout)
{
    boost::system::error_code ec;
    std::size_t length = 0;
    boost::asio::async_write(*serial_port, boost::asio::buffer(buffer, size),
                             [&](const boost::system::error_code& result_error, std::size_t result_n)
                             {
                                 ec = result_error;
                                 length = result_n;
                             });

    bool timedout = runFor(timeout);

    if (checkError(ec, timedout, __PRETTY_FUNCTION__)) {
        return -1;
    }

    return length;
}

int Serial::readSome(uint8_t *buffer, std::size_t size)
{
    boost::system::error_code ec;
    std::size_t length = serial_port->read_some(boost::asio::buffer(buffer, size), ec);

    if (checkError(ec, __PRETTY_FUNCTION__)) {
        return -1;
    }

    return length;
}

int Serial::readSome(uint8_t *buffer, std::size_t size, const std::chrono::steady_clock::duration &timeout)
{
    boost::system::error_code ec;
    std::size_t length = 0;
    serial_port->async_read_some(boost::asio::buffer(buffer, size),
                                  [&](const boost::system::error_code& result_error, std::size_t result_n)
                                  {
                                      ec = result_error;
                                      length = result_n;
                                  });

    bool timedout = runFor(timeout);

    if (checkError(ec, timedout, __PRETTY_FUNCTION__)) {
        return -1;
    }

    return length;
}

int Serial::readExactly(uint8_t *buffer, std::size_t size)
{
    boost::system::error_code ec;
    std::size_t length = boost::asio::read(*serial_port, boost::asio::buffer(buffer, size), ec);

    if (checkError(ec, __PRETTY_FUNCTION__)) {
        return -1;
    }

    return length;
}

int Serial::readExactly(uint8_t *buffer, std::size_t size, const std::chrono::steady_clock::duration &timeout)
{
    boost::system::error_code ec;
    std::size_t n = 0;
    boost::asio::async_read(*serial_port, boost::asio::buffer(buffer, size),
                            [&](const boost::system::error_code& result_error, std::size_t result_n)
                            {
                                ec = result_error;
                                n = result_n;
                            });

    bool timedout = runFor(timeout);

    if (checkError(ec, timedout, __PRETTY_FUNCTION__)) {
        return -1;
    }

    return n;
}

int Serial::readUntil(std::string &buffer, const std::string &delim)
{
    boost::system::error_code ec;
    std::size_t length = boost::asio::read_until(*serial_port, boost::asio::dynamic_buffer(buffer), delim, ec);

    if (checkError(ec, __PRETTY_FUNCTION__)) {
        return -1;
    }

    return length;
}

int Serial::readUntil(std::string &buffer, const std::string &delim, const std::chrono::steady_clock::duration &timeout)
{
    boost::system::error_code ec;
    std::size_t n = 0;
    boost::asio::async_read_until(*serial_port, boost::asio::dynamic_buffer(buffer), delim,
                                  [&](const boost::system::error_code& result_error, std::size_t result_n)
                                  {
                                      ec = result_error;
                                      n = result_n;
                                  });

    bool timedout = runFor(timeout);

    if (checkError(ec, timedout, __PRETTY_FUNCTION__)) {
        return -1;
    }

    return n;
}

void Serial::readAsyncStart()
{
    buffer_read = std::make_unique<uint8_t[]>(buffer_read_size);

    std::thread([this]{
        io_context->run();
    }).detach();

    readAsync();
}

void Serial::readAsync()
{
    serial_port->async_read_some(boost::asio::buffer(buffer_read.get(), buffer_read_size),
                                 [&](const boost::system::error_code& result_error, std::size_t result_n)
                                 {
                                     readAsyncRecieve(result_error, result_n);
                                 });
}

void Serial::readAsyncRecieve(const boost::system::error_code& ec, size_t size)
{
    if (!opened) {
        return;
    }
    if (checkError(ec, __PRETTY_FUNCTION__)) {
        return;
    }

    readAsyncProcess(size);
    readAsync();
}

void Serial::readAsyncProcess(size_t size)
{
    std::cout << __PRETTY_FUNCTION__ << ": Recieved " << size << " bytes." << std::endl;
}

bool Serial::runFor(const std::chrono::steady_clock::duration &timeout)
{
    io_context->restart();
    io_context->run_for(timeout);

    if (!io_context->stopped()) {
        serial_port->cancel();
        io_context->run();
        return true;
    }

    return false;
}

bool Serial::checkError(const boost::system::error_code &ec, const std::string &text)
{
    if (ec) {
        if (debug) {
            std::cout << text << ": " << ec.message() << std::endl;
        }
        return true;
    }

    return false;
}

bool Serial::checkError(const boost::system::error_code &ec, bool timedout, const std::string &text)
{
    if (!timedout && ec) {
        if (debug) {
            std::cout << text << ": " << ec.message() << std::endl;
        }
        return true;
    }

    return false;
}
