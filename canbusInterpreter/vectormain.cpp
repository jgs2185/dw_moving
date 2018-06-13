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

#define _CRT_SECURE_NO_WARNINGS

#include <iostream>
#include <signal.h>
#include <fstream>

#include <stdio.h>
#include <stdlib.h>
#include <memory>

#ifdef LINUX
#include <execinfo.h>
#include <unistd.h>
#endif

#include <cstring>
#include <functional>
#include <list>
#include <iomanip>
#include <vector>

#include <chrono>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <algorithm>

#include <framework/ProgramArguments.hpp>
#include <framework/DataPath.hpp>
#include <framework/Log.hpp>

// CORE
#include <dw/core/Logger.h>
#include <dw/core/Context.h>

// HAL
#include <dw/sensors/Sensors.h>
#include <dw/sensors/radar/Radar.h>
#include <dw/sensors/canbus/CAN.h>
#include <dw/sensors/canbus/Interpreter.h>

// simple plugin-based interpreter
#include "interpreter.hpp"

//------------------------------------------------------------------------------
// Variables
//------------------------------------------------------------------------------
static volatile bool gRun = true;


//------------------------------------------------------------------------------
void sig_int_handler(int sig)
{
    (void)sig;

    gRun = false;
}

//------------------------------------------------------------------------------
void printAllSignalValues(dwCANInterpreterHandle_t canParser)
{
    dwStatus status;
    uint32_t num;
    status = dwCANInterpreter_getNumberSignals(&num, canParser);

    if( status==DW_SUCCESS && num > 0)
    {
        float32_t value = 0;
        dwTime_t timestamp = 0;
        const char *name;

        for (uint32_t i = 0; i < num; ++i )
        {
            if(dwCANInterpreter_getSignalName(&name, i, canParser) == DW_SUCCESS)
            {
                if (dwCANInterpreter_getf32(&value, &timestamp, i, canParser) == DW_SUCCESS)
                {
                    if(0 == strcmp(name,TARGET_ID.c_str())){
                        std::cout << " Target ID " << value << " - " << std::endl;}
                    else if(0 == strcmp(name,TARGET_SNR.c_str())){
                        std::cout << " Target SNR " << value << " - " << std::endl;}
                    else if(0 == strcmp(name,DISTANCE.c_str())){
                        std::cout << " Target Distance " << value << " m " << std::endl;}
                    else if(0 == strcmp(name,VELOCITY.c_str())){
                        std::cout << " Velocity " << value << " m/s " << std::endl;}
                    else if(0 == strcmp(name,ANGLE.c_str())){
                        std::cout << " Angle " << value << " degree " << std::endl;}
                    else
                        std::cout << name << ":" << value << (i < num - 1 ? ", " : "") << std::endl;
                } else {
                    std::cout << "no value retrieved" << name << ":" << (i < num - 1 ? ", " : "");
                }
            } else {
                std::cout << "no signal name" << std::endl;
            }	
        }
    }
}

//-------------------------------------------------------------------------------
dwRadarDetection parseCANtarget(dwCANMessage msg)
{
    dwRadarDetection detection;

    if(msg.size > 0){

        detection.radius = (int16_t)( (msg.data[2] << 8) + msg.data[3]) / 100.0;
        detection.azimuth = (int16_t)( (msg.data[6] << 8) + msg.data[7]) / 100.0 * -1;
        detection.radialVelocity = (int16_t)( (msg.data[4] << 8) + msg.data[5]) / 100.0;
        detection.rcs = msg.data[1];
    }

    return detection;

}


//------------------------------------------------------------------------------
int main(int argc, const char **argv)
{
#ifndef WINDOWS
    struct sigaction action = {};
    action.sa_handler = sig_int_handler;

    sigaction(SIGHUP, &action, NULL);  // controlling terminal closed, Ctrl-D
    sigaction(SIGINT, &action, NULL);  // Ctrl-C
    sigaction(SIGQUIT, &action, NULL); // Ctrl-\, clean quit with core dump
    sigaction(SIGABRT, &action, NULL); // abort() called.
    sigaction(SIGTERM, &action, NULL); // kill command
#endif

    gRun = true;

    ProgramArguments arguments(
        {
            ProgramArguments::Option_t("driver", "can.socket"),
            ProgramArguments::Option_t("params", "device=can0"),
            ProgramArguments::Option_t("dbc", (DataPath::get() + "lib/kanza77.dbc").c_str()),
        });

    dwContextHandle_t sdk   = DW_NULL_HANDLE;
    dwSALHandle_t hal       = DW_NULL_HANDLE;

    // create a Logger to log to console
    // we keep the ownership of the logger at the application level
    dwLogger_initialize(getConsoleLoggerCallback(true));
    dwLogger_setLogLevel(DW_LOG_VERBOSE);

    // instantiate Driveworks SDK context
    dwContextParameters sdkParams = {};

    dwInitialize(&sdk, DW_VERSION, &sdkParams);

    // create HAL module of the SDK
    dwSAL_initialize(&hal, sdk);

    // create CAN bus interface
    dwSensorHandle_t canSensor = DW_NULL_HANDLE;
    {
        dwSensorParams params;
        std::string parameterString = arguments.get("params");
        params.parameters           = parameterString.c_str();
        params.protocol = arguments.get("driver").c_str();
        if (dwSAL_createSensor(&canSensor, params, hal) != DW_SUCCESS) {
            std::cout << "Cannot create sensor "
                      << params.protocol << " with " << params.parameters << std::endl;

            dwSAL_release(&hal);
            dwRelease(&sdk);
            dwLogger_release();

            return -1;
        }
    }

    // load the dbc for K77
    dwCANInterpreterHandle_t canParser = DW_NULL_HANDLE;
    if (arguments.has("dbc") && arguments.get("dbc") != "plugin"){
        std::cout << "Create DBC-based CAN message interpreter" << std::endl;

        std::string inputFilePath  = arguments.get("dbc");

        dwStatus result = dwCANInterpreter_buildFromDBC(&canParser, inputFilePath.c_str(), sdk);
        if (result != DW_SUCCESS) {
            std::cout << "Cannot create DBC-based CAN message interpreter" << std::endl;
        }
    }

    gRun = dwSensor_start(canSensor) == DW_SUCCESS;
    bool readyForTargets = false;

    //this vector will be repopulated on a 'per-scan' basis. 
    //there currently is no publishing mechanism implemented.  
    //this should be implemented in the line below
    // } else if(msg.id == 0x43F ){

    std::vector<dwRadarDetection> detections;

    // receive messages
    while (gRun) {
        std::this_thread::yield();

        dwCANMessage msg;

        dwStatus status = dwSensorCAN_readMessage(&msg, 100000, canSensor);


        if (status == DW_TIME_OUT)
            continue;
        if (status == DW_END_OF_STREAM) {
            std::cout << "EndOfStream" << std::endl;
            break;
        }
        
        // pass message to interpreter
        if (status == DW_SUCCESS && canParser) {
            dwCANInterpreter_consume(&msg, canParser);
        }

        // log message
        std::cout << msg.timestamp_us;
        if (status != DW_SUCCESS) {
            std::cout << " ERROR " << dwGetStatusName(status);
        } else {
            //This is a header frame for Kanza-77
            if( msg.id == 0x43E ) {
                readyForTargets = true;
                continue;
            //This is a footer frame for Kanza-77
            } else if( (msg.id == 0x43F) && (readyForTargets) ) {
                //this is where you direct the output of the detections vector.          
                readyForTargets = false;
                continue;
            //Everything else is either a target or a radar ACK
            } else if( readyForTargets ){
                dwRadarDetection received;
                received = parseCANtarget(msg);
                std::cout << "SNR: "      << received.rcs << std::endl;
                std::cout << "distance: " << received.radius << std::endl;
                std::cout << "velocity: " << received.radialVelocity << std::endl;
                std::cout << "angle: "    << received.azimuth << std::endl;
                
                //add the radar detection to the detection vector.
                detections.push_back(received);
            }
        }
        std::cout << std::endl;
    }

    if (canParser != DW_NULL_HANDLE)
        dwCANInterpreter_release(&canParser);

    dwSensor_stop(canSensor);
    dwSAL_releaseSensor(&canSensor);

    // release used objects in correct order
    dwSAL_release(&hal);
    dwRelease(&sdk);
    dwLogger_release();

    return 0;
}
