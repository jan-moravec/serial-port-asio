#ifndef PLATFORM_H
#define PLATFORM_H

#include <vector>
#include <string>

struct PortInfo {
    /// Address of the serial port.
    std::string port;
    /// Description of serial device if available.
    std::string description;
    /// Hardware ID (e.g. VID:PID of USB serial devices) or "n/a" if not available.
    std::string hardware_id;
};

std::vector<PortInfo> listPorts();

#endif // PLATFORM_H
