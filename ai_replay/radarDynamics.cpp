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

#include <iostream>
#include <cstring>
#include <chrono>
#include <thread>

#include "radarDynamics.h"

dwContextHandle_t                    gContext     = DW_NULL_HANDLE;
dwSensorHandle_t                     gCanSensor   = DW_NULL_HANDLE;
dwSensorHandle_t                     gRadar       = DW_NULL_HANDLE;

dwRadarMountPosition gMountPosition {};

dwTime_t gReportingPeriod = 0;
dwRadarVehicleState gDynamicsPacket {};

int8_t gLastVelocitySign = 0;

//#######################################################################################
// Initialize radarDynamics
//#######################################################################################
dwStatus radarDynamics_initialize(dwContextHandle_t &sdk, dwSALHandle_t &sal,
                                dwSensorHandle_t &radar, ProgramArguments &arguments) {

    dwStatus ret = DW_SUCCESS;
    dwSensorParams params;

    std::string parameterString = arguments.get("can-params");
    params.parameters           = parameterString.c_str();
    params.protocol = arguments.get("can-driver").c_str();

    dwRadarProperties properties;
    dwSensorRadar_getProperties(&properties, radar);

    if (properties.inputPacketsPerSecond > 0) {
        gReportingPeriod = (1.0f / properties.inputPacketsPerSecond) * 1000000;
    } else {
        std::cout << "Radar reported an invalid reporting input period" << std::endl;
        return DW_INVALID_ARGUMENT;
    }

    ret = dwSAL_createSensor(&gCanSensor, params, sal);
    if (ret != DW_SUCCESS) {
        std::cout << "Cannot create sensor "
                  << params.protocol << " with " << params.parameters << std::endl;

        return ret;
    }

    // Load radar pose from rigConfiguration to fill mounting position
    std::string rigFile = arguments.get("rig-configuration");
    if (!rigFile.length()) {
        std::cout<<"Could not load rig configuration file"<<std::endl;
        return DW_FAILURE;
    }

    // Rig Configuration filename found
    static const dwVehicle *vehicleProps = DW_NULL_HANDLE;
    dwRigConfigurationHandle_t rigConf = DW_NULL_HANDLE;
    if (dwRigConfiguration_initializeFromFile(&rigConf, sdk, rigFile.c_str()) != DW_SUCCESS) {
        std::cout << "Cannot create open RigConfiguration file " << rigFile
                   << std::endl;
        return DW_FAILURE;
    }

    // Look for radar sensor lookup name
    // Load radar pose from rigConfiguration to fill mounting position
    std::string radarName = arguments.get("radar-name");
    if (!radarName.length()) {
        std::cout<<"Radar name not specified"<<std::endl;
        return DW_FAILURE;
    }

    // Radar name found. Get ID and retrieve pose
    uint32_t radarID = 0;

    dwTransformation radar2Rig;
    if (getSensorIDfromName(radarID, radarName, rigConf)) {
        dwRigConfiguration_getSensorToRigTransformation(&radar2Rig, radarID, rigConf);
    }
    else {
        std::cout << "Cannot create find sensor in RigConfiguration file " << rigFile
                   << std::endl;
        return DW_FAILURE;
    }

    // Get vehicle measurements
    if (DW_SUCCESS != dwRigConfiguration_getVehicle(&vehicleProps, rigConf)) {
        std::cout << "Cannot find vehicle in RigConfiguration file " << rigFile
                   << std::endl;
        return DW_FAILURE;
    }

    // Get sensor ID from Radar IP
    std::string ip = arguments.get("ip");
    if (!ip.length()) {
        std::cout<<"Cannot find radar ip"<<std::endl;
        return DW_FAILURE;
    }

    size_t startPos = ip.rfind(".");

    if (startPos == std::string::npos) {
        std::cout << "Cannot extract IP from parameter string "
                   << std::endl;
        throw std::exception();
    }
    ip = ip.substr(startPos+1);
    uint32_t sensorID = 0;

    try
    {
        sensorID = stoi(ip);
    }
    catch(...)
    {
        std::cout << "Cannot extract sensor ID from radar IP "
                   << std::endl;
        return DW_FAILURE;
    }

    dwBool isReversed = 0;
    if (arguments.get("isReversed") == "true")
        isReversed = 1;

    // get radome damping
    const float32_t DEFAULT_RADOME_DAMPING = 20.0f;
    float32_t radomeDamping = DEFAULT_RADOME_DAMPING;
    std::string radDampStr = arguments.get("radome-damping");
    try
    {
        radomeDamping = std::stof(radDampStr.c_str());
    }
    catch (...)
    {
        std::cout << "RadarDynamics: Cannot extract radome damping from parameter string "
                  << std::endl;
    }
    // check radome damping min and max limits
    if(radomeDamping >= -20.0 && radomeDamping <= 20.0){
        std::cout << "Using radome damping value of: " << radomeDamping
                  << std::endl;
    } else {
        std::cout << "Radome damping value " << radomeDamping
                  << " not in valid range -20.0 to 20.0, using default: " << DEFAULT_RADOME_DAMPING
                  << std::endl;
    }

    gMountPosition = {
        .sensorId = sensorID,
        .radarPosition = radar2Rig,
        .wheelbase = vehicleProps->wheelbase,
        .damping = radomeDamping,
        .isReversed = isReversed,
    };


    // Copy handles for later use
    gRadar = radar;
    gContext = sdk;

    return ret;
}

//#######################################################################################
// Close
//#######################################################################################
void radarDynamics_stop() {
    dwSensor_stop(gCanSensor);
    dwSAL_releaseSensor(&gCanSensor);
}

//#######################################################################################
// Process Loop
//#######################################################################################
void radarDynamics_process(bool *run) {

    dwStatus ret = DW_SUCCESS;

    // start the CAN sensor
    ret = dwSensor_start(gCanSensor);
    if (ret != DW_SUCCESS) {
        std::cout<<"Unable to start CAN sensor"<<std::endl;
        exit(-1);
        *run = false;
        // ToDo: Fix this
    }

    dwTime_t tn;
    dwContext_getCurrentTime(&tn, gContext);

    dwTime_t t0send = tn + gReportingPeriod;

    // ToDo: construct the radarDynamics structure & report to the radar here
    dwSensorRadar_setMountPosition(&gMountPosition, gRadar);

    while (*run) {
        std::this_thread::yield();

        // grab message from CAN and pass it to vehicle controller
        dwCANMessage msg{};
        dwStatus status = dwSensorCAN_readMessage(&msg, 100000, gCanSensor);
        if (status == DW_SUCCESS)
        {
            switch (msg.id) {
            case CAN_MSG_ID_STEERING_REPORT :
                parseSteeringMsg(&msg);
                break;
            case CAN_MSG_ID_GEAR_REPORT :
                parseGearMsg(&msg);
                break;
            case CAN_MSG_ID_REPORT_ACCEL :
                parseImuAccelMsg(&msg);
                break;
            case CAN_MSG_ID_REPORT_GYRO :
                parseImuGyroMsg(&msg);
                break;
            default:
                break;
                // Unrelated CAN message. Do not parse
            }
        }

        dwContext_getCurrentTime(&tn, gContext);

        if (tn >= t0send) {
            dwSensorRadar_setVehicleState(&gDynamicsPacket, gRadar);
            t0send += gReportingPeriod;
        }
    }
}
//#######################################################################################
// Helper functions
//#######################################################################################
bool getSensorIDfromName(uint32_t &sensorID, const std::string &queryName, const dwRigConfigurationHandle_t rigConf)
{
    sensorID = 0;
    bool found = false;
    const char *sensorName;

    while (dwRigConfiguration_getSensorName(&sensorName, sensorID++, rigConf) == DW_SUCCESS) {
        if (strcmp(sensorName, queryName.c_str()) == 0) {
            found = true;
            sensorID--;
            break;
        }
    }
    return found;
}

//#######################################################################################
// CAN Message parsers
//#######################################################################################
void parseSteeringMsg(const dwCANMessage *msg)
{
    const steeringReport *ptr = reinterpret_cast<const steeringReport*>(msg->data);
    //range: 0.0 to 655.35, LSB: 0.01 km/h = 0.01 / 3.6 m/s
    gDynamicsPacket.velocity  = static_cast<float32_t>(gLastVelocitySign) * static_cast<float32_t>(ptr->SPEED) * (0.01f / 3.6f);
}

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
void parseImuAccelMsg(const dwCANMessage *msg)
{
    const accelReport *ptr = reinterpret_cast<const accelReport *>(msg->data);
    //range: -327.68 to 327.67, LSB: 0.01 m/s^2
    gDynamicsPacket.acceleration = static_cast<float32_t>(ptr->accel_long) * 0.01f;
    //range: -327.68 to 327.67, LSB: 0.01 m/s^2
    gDynamicsPacket.lateralAcceleration = static_cast<float32_t>(ptr->accel_lat) * 0.01f;
}

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
void parseImuGyroMsg(const dwCANMessage *msg)
{
    const gyroReport *ptr = reinterpret_cast<const gyroReport *>(msg->data);
    //range: -6.5536 to 6.5534, LSB: 0.0002 rad/2
    gDynamicsPacket.yawRate = static_cast<float32_t>(ptr->gyro_yaw) * 0.0002f;
}

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
void parseGearMsg(const dwCANMessage *msg)
{
    const gearReport *ptr = reinterpret_cast<const gearReport *>(msg->data);
    if (ptr->STATE == REVERSE_GEAR)
        gLastVelocitySign = -1;
    else
        gLastVelocitySign = 1;
}
