#include "platform.h"

// UNIX specific functions
#if defined(__unix__) || defined(__unix)

#include <sstream>
#include <iostream>
#include <fstream>
#include <glob.h>
#include <sys/types.h>
#include <sys/stat.h>

static std::vector<std::string> searchPathname(const std::vector<std::string> &patterns)
{
    std::vector<std::string> paths;
    glob_t glob_results{};

    for (const std::string &pattern : patterns) {
        glob(pattern.c_str(), GLOB_APPEND, NULL, &glob_results);
    }

    for (unsigned i = 0; i < glob_results.gl_pathc; i++) {
        paths.push_back(glob_results.gl_pathv[i]);
    }

    globfree(&glob_results);
    return paths;
}

static std::string getBasename(const std::string &path)
{
    size_t pos = path.rfind("/");

    if (pos == std::string::npos) {
        return path;
    }

    return std::string(path, pos+1, std::string::npos);
}

static std::string getDirname(const std::string &path)
{
    size_t pos = path.rfind("/");

    if (pos == std::string::npos) {
        return path;
    } else if(pos == 0) {
        return "/";
    }

    return std::string(path, 0, pos);
}

static bool pathExists(const std::string &path)
{
    struct stat sb;

    if (stat(path.c_str(), &sb) == 0) {
        return true;
    }

    return false;
}

static std::string getRealpath(const std::string &path)
{
    char *real_path = realpath(path.c_str(), NULL);
    std::string result;

    if (real_path != NULL) {
        result = real_path;
        free(real_path);
    }

    return result;
}

static std::string readLine(const std::string &file)
{
    std::ifstream ifs(file.c_str(), std::ifstream::in);
    std::string line;

    if (ifs) {
        std::getline(ifs, line);
    }

    return line;
}

static std::string usb_sysfs_friendly_name(const std::string &sys_usb_path)
{
    std::string manufacturer = readLine(sys_usb_path + "/manufacturer");
    std::string product = readLine(sys_usb_path + "/product");
    std::string serial = readLine(sys_usb_path + "/serial");

    std::string result;

    if (!manufacturer.empty()) {
        result += manufacturer + " ";
    }
    if (!product.empty()) {
        result += product + " ";
    }
    if (!serial.empty()) {
        result += serial;
    }

    return result;
}

std::string usb_sysfs_hw_string(const std::string &sysfs_path)
{
    std::string serial_number = readLine(sysfs_path + "/serial");

    if (!serial_number.empty()) {
        serial_number = "SNR=" + serial_number;
    }

    std::string vid = readLine(sysfs_path + "/idVendor");
    std::string pid = readLine(sysfs_path + "/idProduct");

    std::string result = "USB VID:PID=" + vid + ":" + pid;

    if (!serial_number.empty()) {
        result += " " + serial_number;
    }

    return result;
}

static PortInfo getSysfsInfo(const std::string &device_path)
{
    std::string device_name = getBasename(device_path);
    std::string friendly_name;
    std::string hardware_id;
    std::string sys_device_path = "/sys/class/tty/" + device_name + "/device";

    if (device_name.compare(0, 6, "ttyUSB") == 0) {
        sys_device_path = getDirname(getDirname(getRealpath(sys_device_path)));

        if (pathExists(sys_device_path)) {
            friendly_name = usb_sysfs_friendly_name(sys_device_path);
            hardware_id = usb_sysfs_hw_string(sys_device_path);
        }

    } else if (device_name.compare(0,6,"ttyACM") == 0) {
        sys_device_path = getDirname(getRealpath(sys_device_path));

        if ( pathExists(sys_device_path)) {
            friendly_name = usb_sysfs_friendly_name(sys_device_path);
            hardware_id = usb_sysfs_hw_string(sys_device_path);
        }

    } else {
        std::string sys_id_path = sys_device_path + "/id";

        if (pathExists(sys_id_path)) {
            hardware_id = readLine(sys_id_path);
        }

    }

    if (friendly_name.empty()) {
        friendly_name = device_name;
    }


    if (hardware_id.empty()) {
        hardware_id = "n/a";
    }

    PortInfo port;
    port.port = getBasename(device_path);
    port.device = device_path;
    port.description = friendly_name;
    port.hardware_id = hardware_id;

    return port;
}

std::vector<PortInfo> GetSerialPorts()
{
    std::vector<PortInfo> ports;

    std::vector<std::string> patterns;
    patterns.push_back("/dev/ttyACM*");
    //patterns.push_back("/dev/ttyS*");
    patterns.push_back("/dev/ttyUSB*");
    //patterns.push_back("/dev/tty.*");
    //patterns.push_back("/dev/cu.*");

    std::vector<std::string> paths = searchPathname(patterns);

    for (const std::string &path: paths) {
        PortInfo port = getSysfsInfo(path);
        ports.push_back( port );
    }

    return ports;
}
#endif // UNIX and APPLE

// Windows specific functions
#if defined(_WIN32)

#include <iostream>
#include <tchar.h>
#include <windows.h>
#include <setupapi.h>
#include <initguid.h>
#include <devguid.h>

static const DWORD port_name_max_length = 256;
static const DWORD friendly_name_max_length = 256;
static const DWORD hardware_id_max_length = 256;

// Convert a wide Unicode string to an UTF8 string
static std::string wstringUtf16TostringUtf8(const std::wstring &wstr)
{
    int size_needed = WideCharToMultiByte(CP_UTF8, 0, &wstr[0], (int)wstr.size(), NULL, 0, NULL, NULL);
    std::string str( size_needed, 0 );
    WideCharToMultiByte(CP_UTF8, 0, &wstr[0], (int)wstr.size(), &str[0], size_needed, NULL, NULL);
    return str;
}

std::vector<PortInfo> GetSerialPorts()
{
    std::vector<PortInfo> ports;

    HDEVINFO device_info_set = SetupDiGetClassDevs((const GUID *) &GUID_DEVCLASS_PORTS, NULL, NULL, DIGCF_PRESENT);
    unsigned int device_info_set_index = 0;
    SP_DEVINFO_DATA device_info_data;
    device_info_data.cbSize = sizeof(SP_DEVINFO_DATA);

    while(SetupDiEnumDeviceInfo(device_info_set, device_info_set_index, &device_info_data)) {
        device_info_set_index++;

        // Get port name
        HKEY hkey = SetupDiOpenDevRegKey(device_info_set, &device_info_data, DICS_FLAG_GLOBAL, 0, DIREG_DEV, KEY_READ);

        TCHAR port_name[port_name_max_length];
        DWORD port_name_length = port_name_max_length;

        LONG return_code = RegQueryValueEx(hkey, _T("PortName"), NULL, NULL, (LPBYTE)port_name, &port_name_length);
        RegCloseKey(hkey);

        if (return_code != EXIT_SUCCESS) {
            continue;
        }

        if (port_name_length > 0 && port_name_length <= port_name_max_length) {
            port_name[port_name_length-1] = '\0';
        } else {
            port_name[0] = '\0';
        }

        // Ignore parallel ports
        if(_tcsstr(port_name, _T("LPT")) != NULL) {
            continue;
        }

        // Get port friendly name
        TCHAR friendly_name[friendly_name_max_length];
        DWORD friendly_name_actual_length = 0;
        BOOL got_friendly_name = SetupDiGetDeviceRegistryProperty(device_info_set, &device_info_data, SPDRP_FRIENDLYNAME, NULL, (PBYTE)friendly_name, friendly_name_max_length, &friendly_name_actual_length);

        if(got_friendly_name == TRUE && friendly_name_actual_length > 0) {
            friendly_name[friendly_name_actual_length-1] = '\0';
        } else {
            friendly_name[0] = '\0';
        }

        // Get hardware ID
        TCHAR hardware_id[hardware_id_max_length];
        DWORD hardware_id_actual_length = 0;
        BOOL got_hardware_id = SetupDiGetDeviceRegistryProperty(device_info_set, &device_info_data, SPDRP_HARDWAREID, NULL, (PBYTE)hardware_id, hardware_id_max_length, &hardware_id_actual_length);

        if(got_hardware_id == TRUE && hardware_id_actual_length > 0) {
            hardware_id[hardware_id_actual_length-1] = '\0';
        } else {
            hardware_id[0] = '\0';
        }

#ifdef UNICODE
        std::string portName = wstringUtf16TostringUtf8(port_name);
        std::string friendlyName = wstringUtf16TostringUtf8(friendly_name);
        std::string hardwareId = wstringUtf16TostringUtf8(hardware_id);
#else
        std::string portName = port_name;
        std::string friendlyName = friendly_name;
        std::string hardwareId = hardware_id;
#endif

        PortInfo port;
        port.port = portName;
        port.description = friendlyName;
        port.hardware_id = hardwareId;

        std::string prefix = "\\\\.\\";
        if (port.port.compare(0, prefix.size(), prefix) == 0) {
            port.device = port.port;
        } else {
            port.device = prefix + port.port;
        }

        ports.push_back(port);
    }

    SetupDiDestroyDeviceInfoList(device_info_set);

    return ports;
}
#endif // WINDOWS
