#ifndef PLATFORM_H
#define PLATFORM_H

#include <vector>
#include <string>

struct PortInfo {
    /// Port name
    std::string port;
    /// Port device, used for creating Serial
    std::string device;
    /// Port description
    std::string description;
    /// Port hardware ID
    std::string hardware_id;
};

/// Function returns all available serial ports.
/// This function has different implementation for each platform (Windows or Linux).
std::vector<PortInfo> GetSerialPorts();

#endif // PLATFORM_H
