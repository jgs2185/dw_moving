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
// Copyright (c) 2017 NVIDIA Corporation. All rights reserved.
//
// NVIDIA Corporation and its licensors retain all intellectual property and proprietary
// rights in and to this software and related documentation and any modifications thereto.
// Any use, reproduction, disclosure or distribution of this software and related
// documentation without an express license agreement from NVIDIA Corporation is
// strictly prohibited.
//
/////////////////////////////////////////////////////////////////////////////////////////

#ifndef RADAR_DYNAMICS_H_
#define RADAR_DYNAMICS_H_

#include <framework/ProgramArguments.hpp>

// CORE
#include <dw/core/Logger.h>
#include <dw/core/Context.h>

// SAL
#include <dw/sensors/Sensors.h>
#include <dw/sensors/radar/Radar.h>
#include <dw/sensors/canbus/CAN.h>

//RIG CONFIGURATION
#include <dw/rigconfiguration/RigConfiguration.h>

#define REVERSE_GEAR 2

dwStatus radarDynamics_initialize(dwContextHandle_t &sdk, dwSALHandle_t &sal,
                                dwSensorHandle_t &radar, ProgramArguments &arguments);

void radarDynamics_process(bool *run);
void radarDynamics_stop();
bool getSensorIDfromName(uint32_t &sensorID, const std::string &queryName,
                         const dwRigConfigurationHandle_t rigConf);
// CAN parsers
void parseSteeringMsg(const dwCANMessage *msg);
void parseImuAccelMsg(const dwCANMessage *msg);
void parseImuGyroMsg(const dwCANMessage *msg);
void parseGearMsg(const dwCANMessage *msg);

#pragma pack(push, 1)
// CAN message structs
typedef struct {
    int16_t ANGLE;
    int16_t CMD;
    uint16_t SPEED;
    int8_t TORQUE;
    uint8_t ENABLED : 1;
    uint8_t OVERRIDE : 1;
    uint8_t DRIVER : 1;
    uint8_t FLTWDC : 1;
    uint8_t FLTBUS1 : 1;
    uint8_t FLTBUS2 : 1;
    uint8_t FLTCAL : 1;
    uint8_t FLTCON : 1;
} steeringReport;

typedef struct {
    int16_t accel_lat;
    int16_t accel_long;
    int16_t accel_vert;
} accelReport;

typedef struct {
    int16_t gyro_roll;
    int16_t gyro_yaw;
} gyroReport;

typedef struct {
    uint8_t STATE : 3;
    uint8_t OVERRIDE : 1;
    uint8_t CMD : 3;
    uint8_t FLTBUS : 1;
} gearReport;
#pragma pack(pop)

typedef enum {
    CAN_MSG_ID_STEERING_REPORT      = 0x065,
    CAN_MSG_ID_GEAR_REPORT          = 0x067,
    CAN_MSG_ID_REPORT_ACCEL         = 0x06B,
    CAN_MSG_ID_REPORT_GYRO          = 0x06C,
} canMessageID;

#endif // RADAR_DYNAMICS_H_
