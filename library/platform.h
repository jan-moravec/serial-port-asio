#ifndef PLATFORM_H
#define PLATFORM_H

#include <vector>
#include <string>

struct PortInfo {
    /// Port name, human readible form
    std::string port;
    /// Port device, used for creating Serial
    std::string device;
    std::string description;
    std::string hardware_id;
};

std::vector<PortInfo> listPorts();

#endif // PLATFORM_H
