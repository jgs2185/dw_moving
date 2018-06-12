/////////////////////////////////////////////////////////////////////////////////////////
// This code contains NVIDIA Confidential Information and is disclosed
// under the Mutual Non-Disclosure Agreement.
//
// Notice
// ALL NVIDIA DESIGN SPECIFICATIONS AND CODE ("MATERIALS") ARE PROVIDED "AS IS" NVIDIA MAKES
// NO REPRESENTATIONS, WARRANTIES, EXPRESSED, IMPLIED, STATUTORY, OR OTHERWISE WITH RESPECT TO
// THE MATERIALS, AND EXPRESSLY DISCLAIMS ANY IMPLIED WARRANTIES OF NONINFRINGEMENT,
// MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE.
//
// NVIDIA Corporation assumes no responsibility for the consequences of use of such
// information or for any infringement of patents or other rights of third parties that may
// result from its use. No license is granted by implication or otherwise under any patent
// or patent rights of NVIDIA Corporation. No third party distribution is allowed unless
// expressly authorized by NVIDIA.  Details are subject to change without notice.
// This code supersedes and replaces all information previously supplied.
// NVIDIA Corporation products are not authorized for use as critical
// components in life support devices or systems without express written approval of
// NVIDIA Corporation.
//
// Copyright (c) 2015-2016 NVIDIA Corporation. All rights reserved.
//
// NVIDIA Corporation and its licensors retain all intellectual property and proprietary
// rights in and to this software and related documentation and any modifications thereto.
// Any use, reproduction, disclosure or distribution of this software and related
// documentation without an express license agreement from NVIDIA Corporation is
// strictly prohibited.
//
/////////////////////////////////////////////////////////////////////////////////////////

#ifndef SAMPLES_SENSORS_CANBUS_LOGGER_INTERPRETER_HPP__
#define SAMPLES_SENSORS_CANBUS_LOGGER_INTERPRETER_HPP__


#include <dw/sensors/canbus/interpreter/VehicleData.h>
#include <dw/sensors/canbus/CAN.h>

#include <cstddef>
#include <string>
#include <string.h>

// -----------------------------------------------------------------------------
// CAN bus codes
// A set of virtual CAN bus codes to simulate virtual car
// -----------------------------------------------------------------------------
static std::string CAN_CAR_SPEED    = "M_SPEED.CAN_CAR_SPEED";
static std::string CAN_CAR_STEERING = "M_STEERING.CAN_CAR_STEERING";
static std::string TARGET_ID = "TRACK_TARGET.TARGET_ID";
static std::string TARGET_SNR = "TRACK_TARGET.TARGET_SNR";
static std::string DISTANCE = "TRACK_TARGET.DISTANCE";
static std::string VELOCITY = "TRACK_TARGET.VELOCITY";
static std::string ANGLE = "TRACK_TARGET.ANGLE";

// -----------------------------------------------------------------------------
void smpl_addMessage(const dwCANMessage *msg, void *data);

uint32_t smpl_getNumAvailableSignals(void *data);
dwBool smpl_getSignalInfo(const char **name, dwTrivialDataType *type, dwCANVehicleData *data,
                          uint32_t idx, void *userData);

dwBool smpl_getDataf32(float32_t *value, dwTime_t* timestamp, uint32_t idx, void *data);
dwBool smpl_getDatai32(int32_t *value, dwTime_t* timestamp, uint32_t idx, void *data);

#endif // SAMPLES_SENSORS_CANBUS_LOGGER_INTERPRETER_HPP__
