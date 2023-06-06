#pragma once

extern "C" {
#include <libARController/ARCONTROLLER_Device.h>
#include <libARDiscovery/ARDiscovery.h>
#include <libARSAL/ARSAL.h>
}

#include <string>
#include <type_traits>

namespace bebop_driver {
class Bebop {
   private:
    ARSAL_Sem_t stateSem;
    ARDISCOVERY_Device_t* device = nullptr;
    ARCONTROLLER_Device_t* deviceController = nullptr;
    bool is_connected = false;

   public:
    Bebop();
    ~Bebop();
    void connect(const std::string& bebop_ip);
    void takeOff(void);
    void land(void);
    void emergency(void);
    void flatTrim(void);
    void navigateHome(bool start_stop);
    void animationFlip(uint8_t anim_id);
    void move(double roll, double pitch, double gaz_speed, double yaw_speed);
    void moveCamera(double titl, double pan);

    void throwOnInternalError(const std::string& message);
    void throwOnCtrlError(const eARCONTROLLER_ERROR& error,
			  const std::string& message);
};
}  // namespace bebop_driver
