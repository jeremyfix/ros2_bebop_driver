/**
Software License Agreement (BSD)

\file      ardrone3_state_callbacks.hpp
\authors   Jeremy Fix <jeremy.fix@centralesupelec.fr>
\copyright Copyright (c) 2022, CentraleSupélec, All rights reserved. Based on
the work of Mani Monajjemi bebop_autonomy

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
#pragma once

#include <ctime>
#include <mutex>
#include <string>

extern "C" {
#include <libARController/ARCONTROLLER_Dictionary.h>
}

namespace bebop_driver {

using clock_type = std::chrono::high_resolution_clock;
using time_point = std::chrono::time_point<clock_type>;
class Ardrone3PilotingStateAttitudeChanged {
   private:
    ARCONTROLLER_DICTIONARY_ARG_t* arg;
    mutable std::mutex mutex;
    std::tuple<float, float, float> roll_pitch_yaw;
    time_point time;  // The time of the last update
    const std::string frame_id = "base_link";

   public:
    Ardrone3PilotingStateAttitudeChanged();
    void set(const ARCONTROLLER_DICTIONARY_ARG_t* arguments);
    std::tuple<std::string, time_point, float, float, float> get(void) const;
};

class Ardrone3PilotingStateSpeedChanged {
   private:
    ARCONTROLLER_DICTIONARY_ARG_t* arg;
    mutable std::mutex mutex;
    std::tuple<float, float, float> speedx_speedy_speedz;
    time_point time;  // The time of the last update
    const std::string frame_id = "base_link";

   public:
    Ardrone3PilotingStateSpeedChanged();
    void set(const ARCONTROLLER_DICTIONARY_ARG_t* arguments);
    std::tuple<std::string, time_point, float, float, float> get(void) const;
};

}  // namespace bebop_driver
