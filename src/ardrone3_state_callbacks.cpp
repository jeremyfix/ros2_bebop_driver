/**
Software License Agreement (BSD)

\file      aredrone3_state_callbacks.cpp
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

#include "ros2_bebop_driver/ardrone3_state_callbacks.hpp"
extern "C" {
#include <libARController/ARCONTROLLER_Feature.h>
}

#include <mutex>

namespace bebop_driver {
void Ardrone3PilotingStateAttitudeChanged::set(
    const ARCONTROLLER_DICTIONARY_ARG_t *arguments) {
    std::unique_lock<std::mutex> lock(mutex);
    time = clock_type::now();

    arg = NULL;
    HASH_FIND_STR(
	arguments,
	ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGSTATE_ATTITUDECHANGED_ROLL,
	arg);
    if (arg) std::get<0>(roll_pitch_yaw) = arg->value.Float;

    HASH_FIND_STR(
	arguments,
	ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGSTATE_ATTITUDECHANGED_PITCH,
	arg);
    if (arg) std::get<1>(roll_pitch_yaw) = arg->value.Float;

    HASH_FIND_STR(
	arguments,
	ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGSTATE_ATTITUDECHANGED_YAW,
	arg);
    if (arg) std::get<2>(roll_pitch_yaw) = arg->value.Float;
}

std::tuple<std::string, time_point, float, float, float>
Ardrone3PilotingStateAttitudeChanged ::get(void) {
    std::unique_lock<std::mutex> lock(mutex);

    return std::tuple_cat(std::make_tuple(frame_id, time), roll_pitch_yaw);
}

void Ardrone3PilotingStateSpeedChanged::set(
    const ARCONTROLLER_DICTIONARY_ARG_t *arguments) {
    std::unique_lock<std::mutex> lock(mutex);
    time = clock_type::now();

    arg = NULL;
    HASH_FIND_STR(
	arguments,
	ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGSTATE_SPEEDCHANGED_SPEEDX,
	arg);
    if (arg) std::get<0>(speedx_speedy_speedz) = arg->value.Float;

    HASH_FIND_STR(
	arguments,
	ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGSTATE_SPEEDCHANGED_SPEEDY,
	arg);
    if (arg) std::get<1>(speedx_speedy_speedz) = arg->value.Float;

    HASH_FIND_STR(
	arguments,
	ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGSTATE_SPEEDCHANGED_SPEEDZ,
	arg);
    if (arg) std::get<2>(speedx_speedy_speedz) = arg->value.Float;
}

std::tuple<std::string, time_point, float, float, float>
Ardrone3PilotingStateSpeedChanged ::get(void) {
    std::unique_lock<std::mutex> lock(mutex);

    return std::tuple_cat(std::make_tuple(frame_id, time),
			  speedx_speedy_speedz);
}
}  // namespace bebop_driver
