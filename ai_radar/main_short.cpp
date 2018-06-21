//looking at /usr/local/driveworks-0.6/samples/src/dnn/sample_object_detector/main.cpp
//and /usr/local/driveworks-0.6/samples/src/sensors/imu/main.cpp


#include <stdio.h>
#include <stdlib.h>
#include <signal.h>

#include <iostream>
#include <fstream>
#include <memory>
#include <cmath>
#include <cstring>
#include <functional>
#include <list>
#include <iomanip>
#include <chrono>

#include <framework/ProgramArguments.hpp>
#include <framework/DataPath.hpp>
#include <framework/Log.hpp>

// CORE
#include <dw/core/Context.h>
#include <dw/core/Logger.h>

// SAL
#include <dw/sensors/Sensors.h>
#include <dw/sensors/radar/Radar.h>

// Radar drivers
#include "radar.cpp"

//------------------------------------------------------------------------------
// Variables
//------------------------------------------------------------------------------

dwContextHandle_t gSdk                   = DW_NULL_HANDLE;
dwSALHandle_t gSal                       = DW_NULL_HANDLE;
dwSensorHandle_t gRadarSensor            = DW_NULL_HANDLE;
dwContextHandle_t context                = DW_NULL_HANDLE;

bool gRecordedRadar = false;


//#######################################################################################
void initDriveworks()
{
    // create a Logger to log to console
    // we keep the ownership of the logger at the application level
    dwLogger_initialize(getConsoleLoggerCallback(true));
    dwLogger_setLogLevel(DW_LOG_VERBOSE);

    // instantiate Driveworks SDK context
    dwContextParameters sdkParams = {};
/*
#ifdef VIBRANTE
    sdkParams.eglDisplay = gWindow->getEGLDisplay();
#endif
*/
    dwInitialize(&gSdk, DW_VERSION, &sdkParams);
}

//#######################################################################################
void initRadars(ProgramArguments &arguments)
{
//    // create sensor abstraction layer
//    dwSAL_initialize(&gSal, gSdk);

        // create Radar interface
        gRadarSensor = DW_NULL_HANDLE;
        dwRadarProperties      gRadarProperties = {};
        {
            dwSensorParams params;

            std::string parameterString;
            std::string protocolString;

            if (strcmp(arguments.get("protocol").c_str(), "") != 0) {
                protocolString = arguments.get("protocol");

                if (protocolString == "radar.virtual")
                    gRecordedRadar = true;
                else
                    gRecordedRadar = false;
            }

            if (strcmp(arguments.get("params").c_str(), "") != 0)
                parameterString = arguments.get("params");

            if(protocolString.empty() || parameterString.empty())
            {
                std::cout << "INVALID PARAMETERS" << std::endl;
                exit(-1);
            }

            params.protocol = protocolString.c_str();
            params.parameters = parameterString.c_str();
            dwStatus created = dwSAL_createSensor(&gRadarSensor, params, gSal);
            if( created == DW_SUCCESS)
            {
                // Get Radar properties
                if (dwSensorRadar_getProperties(&gRadarProperties, gRadarSensor) == DW_SUCCESS)
                {
                    if (gRadarProperties.scansPerSecond != 0)
                    {
                        // Enable all supported scans
                        for (size_t i = 0; i < DW_RADAR_RETURN_TYPE_COUNT; i++) {
                            for (size_t j = 0; j < DW_RADAR_RANGE_COUNT; j++) {
                                if (!gRadarProperties.supportedScanTypes[i][j])
                                    continue;

                                dwRadarScanType type = {
                                    .returnType = static_cast<dwRadarReturnType>(i),
                                    .range = static_cast<dwRadarRange>(j),
                                };

                                dwSensorRadar_toggleScanType(true, type, gRadarSensor);
                            }
                        }
                    }
                    else
                        throw std::runtime_error("In Radar Properties - packetsPerSecond is 0");
                }
                else
                    throw std::runtime_error("Could not read radar properties");
            }
            else
                if(created == DW_INVALID_ARGUMENT){std::cout << "invalid argument" << std::endl;}
                else if(created == DW_INVALID_HANDLE){std::cout << "invalid handle" << std::endl;}
                else if(created == DW_SAL_NO_DRIVER_FOUND){std::cout << "no driver" << std::endl;}
                else if(created == DW_SAL_SENSOR_ERROR){std::cout << "sensor error" << std::endl;}
                else{
                    std::cout << created << std::endl;
                }
                throw std::runtime_error("Sensor Initialization Failed");
        }

}

//#######################################################################################
void init(ProgramArguments &arguments)
{
    initDriveworks();

    initRadars(arguments);

}


//#######################################################################################


int main(int argc, const char** argv)
{

    Radar my_radar_test("k77",0.0,0.0,0.0); 

    my_radar_test.init();

    my_radar_test.activate();
/*
    std::string dynamicsParams;
    dynamicsParams = "can-driver=can.socket"
                     ",can-params=device=can0"
                     ",rig-configuration=wwdc_rig.xml"
                     ",radar-name=FL"
                     ",isReversed=false"
                     ",radome-damping=0.0";

    ProgramArguments arguments(argc, argv,
    {
        ProgramArguments::Option_t("protocol", "radar.socket"),
        ProgramArguments::Option_t("params","device=CUSTOM, can-driver=can.socket,can-params=device=can0"),
        ProgramArguments::Option_t("enable-dynamics", "false"),
        ProgramArguments::Option_t("dynamics-params", dynamicsParams.c_str()),
    },
    "Radar replay sample which playback .bin video streams in a GL window.");
*/

    std::string dynamicsParams;
    dynamicsParams = "can-driver=can.socket"
                     ",can-params=device=can0"
                     ",rig-configuration=wwdc_rig.xml"
                     ",radar-name=FL"
                     ",isReversed=false"
                     ",radome-damping=0.0";

    ProgramArguments arguments(argc, argv,
    {
        ProgramArguments::Option_t("protocol", "radar.virtual"),
        ProgramArguments::Option_t("params", ("file=" + DataPath::get() + "/samples/sensors/radar/conti/radar_0.bin").c_str()),
        ProgramArguments::Option_t("enable-dynamics", "false"),
        ProgramArguments::Option_t("dynamics-params", dynamicsParams.c_str()),
    },
    "Radar replay sample which playback .bin video streams in a GL window.");



    init(arguments);
/*
    for(int x = 0; x < 10; x++)
    {
        my_radar_test.get_scan();
        my_radar_test.print_scan_info();
    }
*/
    return 0;
}
