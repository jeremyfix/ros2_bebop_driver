/**
Software License Agreement (BSD)

\file      bebop.cpp
\authors   Jeremy Fix <jeremy.fix@centralesupelec.fr>
\copyright Copyright (c) 2022, CentraleSupélec, All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.
 * Neither the name of Autonomy Lab nor the names of its contributors may be
used to endorse or promote products derived from this software without specific
prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WAR- RANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, IN- DIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "ros2_bebop_driver/bebop.hpp"

#include "ros2_bebop_driver/BebopArdrone3Config.h"
/* #include "ros2_bebop_driver/bebop_video_decoder.h" */

// include all callback wrappers
#include "ros2_bebop_driver/ardrone3_setting_callbacks.h"
#include "ros2_bebop_driver/ardrone3_state_callbacks.h"
#include "ros2_bebop_driver/common_state_callbacks.h"

namespace bebop_driver {

Bebop::Bebop() { ARSAL_Sem_Init(&(stateSem), 0, 0); }

Bebop::~Bebop() {
    ARDISCOVERY_Device_Delete(&device);
    ARCONTROLLER_Device_Delete(&deviceController);

    ARSAL_Sem_Destroy(&(stateSem));
}

void Bebop::connect(const std::string& bebop_ip) {
    eARDISCOVERY_ERROR errorDiscovery = ARDISCOVERY_OK;
    device = ARDISCOVERY_Device_New(&errorDiscovery);
}

void Bebop::takeOff() {}

void Bebop::land() {}
}  // namespace bebop_driver
