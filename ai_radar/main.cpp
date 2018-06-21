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
#include <sstream>

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
#include <dw/sensors/canbus/CAN.h>

//#include "radar.cpp"
#include "radarDecoder.cpp"


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
int main(int argc, const char **argv)
{

    if(_dwRadarDecoder_initialize() == DW_SUCCESS){
        std::cout << "radar initialized" << std::endl;
    }

//    Radar my_radar_test("k77",0.0,0.0,0.0);

//    my_radar_test.init();

//    my_radar_test.activate();


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
            ProgramArguments::Option_t("driver", "can.virtual"),
            ProgramArguments::Option_t("params", (std::string("file=") + DataPath::get() +
                                                  "/samples/sfm/triangulation/canbus.can")
                                                     .c_str()),
            ProgramArguments::Option_t("filter", "000:000"),
            ProgramArguments::Option_t("hwtime", "1"),
            ProgramArguments::Option_t("send_i_understand_implications", "0"),
            ProgramArguments::Option_t("send_id", "6FF"),
	});

    if (!arguments.parse(argc, argv)) {
        std::cout << "Usage: " << argv[0] << std::endl;
        std::cout << "\t--driver=can.virtual \t\t\t: one of the available CAN drivers "
                  << "(see sample_sensors_info)\n";
        std::cout << "\t--params=file=canbus.can,arg2=value\t: comma separated "
                  << "key=value parameters for the sensor "
                  << "(see sample_sensor_info for a set of supported parameters)\n";
        std::cout << "\t--send_i_understand_implications=10\t: If > 0 random CAN messages with given delay in msec will be sent"
                  << "\n"
                  << "                 \t\t\t\t: WARNING - do not use this functionality in a production "
                  << "environment, sending routines should be used to test the connectivity in a separated "
                  << "environment only! \n";
        std::cout << "\t--send_id=6FF \t\t\t\t: CAN message ID when sending messages (see send_i_understand_implications)\n";
        std::cout << "\t--hwtime=0 \t\t\t\t: To disable hardware timestamps"
                  << "\n";
        std::cout << "\t--filter=1A2:FFF,100:300,3A:0F,... \t: Pass any number of CAN ids and masks to add a filter"
                  << "\n";
        return -1;
    }

    std::cout << "Program Arguments:\n" << arguments.printList() << std::endl;

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

    if (arguments.has("hwtime") && arguments.get("hwtime") == "0")
    {
        dwSensorCAN_setUseHwTimestamps(DW_FALSE, canSensor);
    }

    dwTime_t sendInterval = 0;
    uint32_t sendTestId = 0x6FF;
    if (arguments.has("send_i_understand_implications") && arguments.get("send_i_understand_implications").length() > 0)
    {
        sendInterval = std::strtoll(arguments.get("send_i_understand_implications").c_str(), nullptr, 10);
    }

    if (arguments.has("send_id") && arguments.get("send_id").length() > 0)
    {
        sendTestId = std::strtoll(arguments.get("send_id").c_str(), nullptr, 16);
    }

    // read out filters
    if (arguments.has("filter"))
    {
        std::vector<uint32_t> ids;
        std::vector<uint32_t> masks;
        std::string filters = arguments.get("filter");
        std::istringstream ss(filters);
        std::string token;

        while(std::getline(ss, token, ','))
        {
            char* dup = new char[token.length()+1];
            std::strcpy(dup, token.c_str());
            uint32_t id = strtoul(strtok(dup, ":"), NULL, 16);
            uint32_t mask = strtoul(strtok(NULL, ":"), NULL, 16);
            delete [] dup;

            ids.push_back(id);
            masks.push_back(mask);
        }

        dwStatus result = dwSensorCAN_setMessageFilter(ids.data(), masks.data(), ids.size(), canSensor);
        if (result != DW_SUCCESS)
        {
            std::cout << "Cannot setup CAN message filters: " << dwGetStatusName(result) << std::endl;
        }
    }

    gRun = dwSensor_start(canSensor) == DW_SUCCESS;

    // send test frame - we use sendTestId.
    // default 6FF, lowest priority as CAN id to prevent interaction with other in-vechicle systems
    if (gRun && sendInterval > 0) {
        std::string message[] = {"Drive", "Works", "SDK"};
        for (const std::string &msg : message) {
            dwCANMessage testMsg;
            testMsg.id           = sendTestId;
            testMsg.size         = std::min((uint16_t)msg.length(), (uint16_t)DW_SENSORS_CAN_MAX_MESSAGE_LEN);
            testMsg.timestamp_us = 0;
            memcpy(testMsg.data, msg.c_str(), testMsg.size);

            dwStatus status = dwSensorCAN_sendMessage(&testMsg, 100000, canSensor);

            if (status != DW_SUCCESS) {
                std::cerr << "Cannot send CAN message: " << dwGetStatusName(status) << std::endl;
                break;
            }

            // small delay
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    dwTime_t tNow;
    dwContext_getCurrentTime(&tNow, sdk);
    dwTime_t tSend = tNow + sendInterval * 1000;

//    dwTime_t lastTime = 0;


    //This is beginning of our radar stuff
    dwStatus headerStatus = DW_FAILURE;
    dwStatus validStatus = DW_FAILURE;

//    uint8_t **fullScanBuffer[128][12] = {0};
    dwRadarScan scanIn;

    while (gRun) {
        std::this_thread::sleep_for(std::chrono::microseconds(10));

        dwCANMessage msg;
        dwStatus status = dwSensorCAN_readMessage(&msg, sendInterval > 0 ? 0 : 100000, canSensor);

        //start calling/checking the messages for header/footer/target frame
        //uint8_t * newbuffer = new uint8_t[msg.size + 12];

        uint8_t id1 = (msg.id & 0xff000000) >> 24;
        uint8_t id2 = (msg.id & 0x00ff0000) >> 16;
        uint8_t id3 = (msg.id & 0x0000ff00) >>  8;
        uint8_t id4 = (msg.id & 0x000000ff)      ;


        uint8_t newarray[12] = {id1, id2, id3, id4, msg.data[0],msg.data[1],msg.data[2],msg.data[3],msg.data[4],msg.data[5],msg.data[6],msg.data[7]};

        const dwRadarScanType testType = {DW_RADAR_RETURN_TYPE_DETECTION,DW_RADAR_RANGE_SHORT};

        dwRadarScanType testType2;

        size_t numRemaining = 1;


        if(headerStatus == DW_FAILURE){
            headerStatus = _dwRadarDecoder_synchronize(newarray, sizeof(newarray), &numRemaining);
        }

        if(headerStatus == DW_SUCCESS) {
            validStatus = _dwRadarDecoder_validatePacket(newarray, sizeof(newarray), &testType2);
            if(validStatus == DW_SUCCESS){
                _dwRadarDecoder_decodePacket(&scanIn,msg.data,msg.size,testType);
                //_dwRadarDecoder_isScanComplete
            } else {
                continue;
            }
//            std::cout << status2 << std::endl;
        } else {
           continue;
        }

        float32_t radiuss = static_cast<dwRadarDetection *>(scanIn.data)->radius;

        std::cout << std::dec << "returned data: " << radiuss << std::endl;

        dwContext_getCurrentTime(&tNow, sdk);


        // send a random message over can bus
        if (tNow >= tSend && sendInterval > 0)
        {
            dwCANMessage testMsg;
            testMsg.id           = sendTestId;
            testMsg.size         = DW_SENSORS_CAN_MAX_MESSAGE_LEN;
            testMsg.timestamp_us = 0;
            uint8_t byte = std::rand() % 255;
            for (int32_t j=0; j < testMsg.size; j++)
                testMsg.data[j] = byte++;

            dwSensorCAN_sendMessage(&testMsg, 1000, canSensor);
            tSend = tNow + sendInterval * 1000;
        }

        if (status == DW_TIME_OUT)
            continue;
        if (status == DW_END_OF_STREAM) {
            std::cout << "EndOfStream" << std::endl;
            gRun = false;
            break;
        }
/*
        // log message
        if (lastTime == 0)
        {
            std::cout << msg.timestamp_us;
        }else {
            std::cout << msg.timestamp_us << " (dt=" <<  static_cast<int64_t>(msg.timestamp_us - lastTime) << ")";
        }

        if (msg.timestamp_us < lastTime)
        {
            std::cout << " ERROR msg out of order - ";
        }
        lastTime = msg.timestamp_us;
        if (status != DW_SUCCESS)
        {
            std::cout << " ERROR " << dwGetStatusName(status);
            break;
        } else {
            std::cout << std::uppercase
                      << " -> " << std::setfill('0') << std::setw(3) << std::hex
                      << msg.id << std::dec << "  [" << msg.size << "] ";
            for (auto i = 0; i < msg.size; i++)
                std::cout << std::setfill('0') << std::setw(2) << std::hex << (int)msg.data[i] << " ";
            std::cout << std::dec;
        }
        std::cout << std::endl;
*/    }

    dwSensor_stop(canSensor);
    dwSAL_releaseSensor(&canSensor);

    // release used objects in correct order
    dwSAL_release(&hal);
    dwRelease(&sdk);
    dwLogger_release();

    _dwRadarDecoder_release();

    return 0;
}
