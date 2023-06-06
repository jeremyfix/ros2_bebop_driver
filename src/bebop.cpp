/**
Software License Agreement (BSD)

\file      bebop.cpp
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

#include "ros2_bebop_driver/bebop.hpp"

#include <cmath>
#include <stdexcept>

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

void Bebop::takeOff(void) {
    throwOnInternalError("TakeOff failed");
    throwOnCtrlError(deviceController->aRDrone3->sendPilotingTakeOff(
			 deviceController->aRDrone3),
		     "Takeoff failed");
}

void Bebop::land(void) {
    throwOnInternalError("Land failed");
    throwOnCtrlError(deviceController->aRDrone3->sendPilotingLanding(
			 deviceController->aRDrone3),
		     "Land failed");
}

void Bebop::emergency(void) {
    throwOnInternalError("Emergency failed");
    throwOnCtrlError(deviceController->aRDrone3->sendPilotingEmergency(
			 deviceController->aRDrone3),
		     "Emergency failed");
}

void Bebop::flatTrim(void) {
    throwOnInternalError("FlatTrim failed");
    throwOnCtrlError(deviceController->aRDrone3->sendPilotingFlatTrim(
			 deviceController->aRDrone3),
		     "FlatTrim failed");
}

void Bebop::navigateHome(bool start_stop) {
    throwOnInternalError("Navigate home failed");
    throwOnCtrlError(deviceController->aRDrone3->sendPilotingNavigateHome(
			 deviceController->aRDrone3, start_stop ? 1 : 0),
		     "Navigate home failed");
}

void Bebop::animationFlip(uint8_t anim_id) {
    throwOnInternalError("Animation failed");
    if (anim_id >= ARCOMMANDS_ARDRONE3_ANIMATIONS_FLIP_DIRECTION_MAX) {
	throw std::runtime_error("Invalid animation id");
    }
    throwOnCtrlError(
	deviceController->aRDrone3->sendAnimationsFlip(
	    deviceController->aRDrone3,
	    static_cast<eARCOMMANDS_ARDRONE3_ANIMATIONS_FLIP_DIRECTION>(
		anim_id % ARCOMMANDS_ARDRONE3_ANIMATIONS_FLIP_DIRECTION_MAX)),
	"Animation flip failed");
}

void Bebop::move(double roll, double pitch, double gaz_speed,
		 double yaw_speed) {
    // TODO(jeremyfix): Bound check
    throwOnInternalError("Move failure");

    // If roll or pitch value are non-zero, enable roll/pitch flag
    const bool do_rp =
	!((std::fabs(roll) < 0.001) && (std::fabs(pitch) < 0.001));

    // If all values are zero, hover
    const bool do_hover = !do_rp && (std::fabs(yaw_speed) < 0.001) &&
			  (std::fabs(gaz_speed) < 0.001);

    if (do_hover) {
	/* ARSAL_PRINT(ARSAL_PRINT_DEBUG, LOG_TAG, "STOP"); */
	throwOnCtrlError(deviceController->aRDrone3->setPilotingPCMD(
			     deviceController->aRDrone3, 0, 0, 0, 0, 0, 0),
			 "Hover failed");
    } else {
	throwOnCtrlError(deviceController->aRDrone3->setPilotingPCMD(
			     deviceController->aRDrone3, do_rp,
			     static_cast<int8_t>(roll * 100.0),
			     static_cast<int8_t>(pitch * 100.0),
			     static_cast<int8_t>(yaw_speed * 100.0),
			     static_cast<int8_t>(gaz_speed * 100.0), 0),
			 "Move failed");
    }
}

// in degrees
void Bebop::moveCamera(double tilt, double pan) {
    throwOnInternalError("Camera Move Failure");
    throwOnCtrlError(deviceController->aRDrone3->setCameraOrientation(
			 deviceController->aRDrone3, static_cast<int8_t>(tilt),
			 static_cast<int8_t>(pan)),
		     "Camera move failed");
}

void Bebop::throwOnInternalError(const std::string& message) {
    if (!is_connected || !deviceController) throw std::runtime_error(message);
}
void Bebop::throwOnCtrlError(const eARCONTROLLER_ERROR& error,
			     const std::string& message) {
    if (error != ARCONTROLLER_OK)
	throw std::runtime_error(
	    message + std::string(ARCONTROLLER_Error_ToString(error)));
}

}  // namespace bebop_driver

