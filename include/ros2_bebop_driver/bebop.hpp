#pragma once

extern "C" {
#include <libARController/ARCONTROLLER_Device.h>
#include <libARDiscovery/ARDiscovery.h>
#include <libARSAL/ARSAL.h>
}

#include <string>

namespace bebop_driver {
class Bebop {
   private:
    ARSAL_Sem_t stateSem;
    ARDISCOVERY_Device_t* device = NULL;
    ARCONTROLLER_Device_t* deviceController = NULL;

   public:
    Bebop();
    ~Bebop();
    void connect(const std::string& bebop_ip);
    void takeOff(void);
    void land(void);
};
}  // namespace bebop_driver
