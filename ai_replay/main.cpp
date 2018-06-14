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

#include <framework/DriveWorksSample.hpp>
#include <framework/MathUtils.hpp>
#include <GLFW/glfw3.h>

#include <signal.h>
#include <string.h>

#include <iostream>
#include <chrono>

#include <thread>
#include <memory>
#include <vector>
#include <sstream>
#include <float.h>
#include <unistd.h>

#include <dw/renderer/Renderer.h>
#include <dw/sensors/Sensors.h>


// CORE
#include <dw/core/Logger.h>
#include <dw/core/Context.h>
#include <dw/renderer/Renderer.h>

// SAL
#include <dw/sensors/Sensors.h>
#include <dw/sensors/radar/Radar.h>

#include "radarDynamics.h"

using namespace dw_samples::common;

//------------------------------------------------------------------------------
// Radar replay sample
// The Radar replay sample demonstrates playback of radar sensor.
//
// The sample opens an X window to play back the provided virtual sensor data file.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Help functions
//------------------------------------------------------------------------------
namespace
{
    template <class T>
    std::string toStr(const T& value)
    {
        std::ostringstream s;
        s.precision(DBL_DIG);
        s << value;
        return s.str();
    }
}

class RadarReplay : public DriveWorksSample
{
private:
    // -----------------------------------------------------------
    // Constants for this radar replay which configure output view
    // -----------------------------------------------------------
    const float WORLD_Z = 0.0f;
    const float WORLD_VELOCITY_LOW_KM_PER_HOUR  = 100.0;
    const float WORLD_VELOCITY_HIGH_KM_PER_HOUR = 200.0f;

    // -----------------------------------------------------------
    // GRID construction and GRID params
    // -----------------------------------------------------------
    const float WORLD_CIRCLE_DR_IN_METERS = 5.0f;
    const float WORLD_GRID_RES_IN_METERS  = 10.0f;
    const float WORLD_GRID_SIZE_IN_METERS_WIDTH  = 220.0f;
    const float WORLD_GRID_SIZE_IN_METERS_HEIGHT = 150.0f;

    // ------------------------------------------------
    // Driveworks Context and SAL adn Renderer
    // ------------------------------------------------
    dwContextHandle_t               context             = DW_NULL_HANDLE;
    dwSALHandle_t                   sal                 = DW_NULL_HANDLE;
    dwRendererHandle_t              renderer            = DW_NULL_HANDLE;

    // ------------------------------------------------
    // Sample specific. Variables for hold message text
    // ------------------------------------------------
    std::string gMessage1;
    std::string gMessage2;
    std::string gMessage3;
    std::string gMessage4;
    std::string gMessage5;
    std::string gMessage6;
    std::string gMessage7;
    std::string gMessage8;
    std::string gMessage9;
    std::string gMessage10;
    std::string gMessage11;


    // -------------------------------------------------------
    // Sample specific. Sensors
    // -------------------------------------------------------
    dwSensorHandle_t       canSensor = DW_NULL_HANDLE;
    dwRadarProperties      gRadarProperties = {};

    // -------------------------------------------------------
    // Sample specific. Various buffer handles for rasterizing
    // -------------------------------------------------------

    dwRenderBufferHandle_t gPointCloud[DW_RADAR_RETURN_TYPE_COUNT][DW_RADAR_RANGE_COUNT];
    dwRenderBufferHandle_t gPointCloudAsLines = DW_NULL_HANDLE;
    dwRenderBufferHandle_t gGroundPlane    = DW_NULL_HANDLE;
    dwRenderBufferHandle_t gGroundCircles  = DW_NULL_HANDLE;
    dwRenderBufferHandle_t gWorldSpaceAxes = DW_NULL_HANDLE;

    char *gPointCloudBuffer[DW_RADAR_RETURN_TYPE_COUNT][DW_RADAR_RANGE_COUNT];

    // ---------------------------------------------------
    // Sample specific. Vector for holding parsed CAN data
    // ---------------------------------------------------
    std::vector<dwRadarDetection> detections;

    // -------------------------------------------------------
    // Sample specific. Various flags for various display modes
    // -------------------------------------------------------
    bool gFullScreen    = false;
    bool gRun           = false;
    bool gPause         = false;
    bool gShowGrid      = true;
    bool gShowText      = true;
    bool gRecordedRadar = false;
    bool gRenderClusterLines = false;
    bool gRenderDetections = true;
    bool gRenderTracks = true;
    bool gRenderClusters = true;
    bool gRenderStatus = true;
    bool readyForTargets = false;
    bool renderReady = true;

    // -------------------------------------------------------
    // Sample specific. Dynamics
    // -------------------------------------------------------
    bool gRunDynamics = false;
    std::thread dynamicsThread;


    // -------------------------------------------------------
    // Sample specific.  For 3D Display
    // -------------------------------------------------------

    double   gFreqMultiplier = 1.0; ///< Frequency muliplier to increase od decrease showed points during radar
                                    ///  scans. When value 1.0 points are reading in radar scan rate.
    dwTime_t gLastTimestamp = 0;
    dwRect gRect;
    float modelview[16];
    float projection[16];

    float eye[3];
    float center[3];
    float up[3];

    float fovRads = (float)DEG2RAD(60.0f);

    // -------------------------------------------------------
    // Sample specific.  Mouse naviagation variables
    // -------------------------------------------------------
    float deltaVerAngle;
    float deltaHorAngle;
    float deltaMove;

    float nearPlane;
    float radius;
    float vertAngle;
    float horAngle;

    bool mouseLeft;
    bool mouseRight;
    int currentX;
    int currentY;

public:

    RadarReplay(const ProgramArguments& args) : DriveWorksSample(args) {}

    void initializeInputDefaults()
    {
        // Initialize 3D view related variables.
        deltaVerAngle = 0.0f;
        deltaHorAngle = 0.0f;
        deltaMove     = 0.0f;

        radius    = 400.0f;
        nearPlane = 170.0f;
        vertAngle = DEG2RAD(89.9f);
        horAngle  = DEG2RAD(179.9);

        mouseLeft  = false;
        mouseRight = false;
        currentX   = -1;
        currentY   = -1;

        // Initialize eye, up for bowl view
        eye[0] = radius * cos(vertAngle) * cos(horAngle);
        eye[1] = radius * cos(vertAngle) * sin(horAngle);
        eye[2] = radius * sin(vertAngle);

        up[0] = 0;
        up[1] = 0;
        up[2] = 1;

        center[0] = 20.;
        center[1] = 0;
        center[2] = 0;
    }

    void onProcessKey(int key) override
    {
        // stop application
        if (key == GLFW_KEY_ESCAPE)
            gRun = false;

        // pause application
        if (key == GLFW_KEY_SPACE)
            gPause = !gPause;

        // reset camera view
        if (key == GLFW_KEY_R)
        {
            initializeInputDefaults();
            gFreqMultiplier = 1.0;
        }

        // change view mode of displayed grid
        if (key == GLFW_KEY_G)
            gShowGrid = !gShowGrid;

        // show hide text messages
        if (key == GLFW_KEY_F1)
            gShowText = !gShowText;

        // decrease frequency of displayed data
        if (key == GLFW_KEY_KP_SUBTRACT)
            gFreqMultiplier *= 0.5;

        // increase frequency of displayed data
        if (key == GLFW_KEY_KP_ADD)
            gFreqMultiplier *= 2;

        // Toggle visualization for detections
        if (key == GLFW_KEY_0)
            gRenderDetections = !gRenderDetections;

        // Toggle visualization for clusters
        if (key == GLFW_KEY_1)
            gRenderClusters = !gRenderClusters;

        // Toggle visualization for tracks
        if (key == GLFW_KEY_2)
            gRenderTracks = !gRenderTracks;

        // Toggle status message rendering
        if (key == GLFW_KEY_3)
            gRenderStatus = !gRenderStatus;

        // Toggle visualization of cluster lines
        if (key == GLFW_KEY_4)
            gRenderClusterLines = !gRenderClusterLines;
    }

    void onMouseDown(int button, float x, float y) override
    {
        // only start motion if the left button is pressed
        if (button == GLFW_MOUSE_BUTTON_LEFT) {
            mouseLeft = true;
            currentX  = (int)floor(x);
            currentY  = (int)floor(y);
        }

        if (button == GLFW_MOUSE_BUTTON_RIGHT) {
            mouseRight = true;
            currentX   = (int)floor(x);
            currentY   = (int)floor(y);
        }
    }

    void onMouseUp(int button, float x, float y) override
    {
        (void)x;
        (void)y;

        // only start motion if the left button is pressed
        if (button == GLFW_MOUSE_BUTTON_LEFT) {
            mouseLeft = false;
            vertAngle += deltaVerAngle;
            horAngle += deltaHorAngle;
            deltaHorAngle = deltaVerAngle = 0;
        }

        if (button == GLFW_MOUSE_BUTTON_RIGHT) {
            mouseRight = false;
            radius += deltaMove;
            deltaMove = 0;
        }
    }
    void onMouseMove(float x, float y) override
    {
        // this will only be true when the left button is down
        if (mouseLeft) {

            // update deltaAngle
            deltaVerAngle = (y - currentY);
            deltaHorAngle = -(x - currentX);

            // scale deltaAngle
            deltaVerAngle *= 0.005f;
            deltaHorAngle *= 0.005f;

            // Limit the vertical angle (0.1 to 89.9 degrees)
            if ((vertAngle + deltaVerAngle) > DEG2RAD(89.9))
                deltaVerAngle = DEG2RAD(89.9) - vertAngle;

            if ((vertAngle + deltaVerAngle) < DEG2RAD(0.1))
                deltaVerAngle = DEG2RAD(0.1) - vertAngle;

            eye[0] = radius * cos(vertAngle + deltaVerAngle) * cos(horAngle + deltaHorAngle);
            eye[1] = radius * cos(vertAngle + deltaVerAngle) * sin(horAngle + deltaHorAngle);
            eye[2] = radius * sin(vertAngle + deltaVerAngle);
        }
    }

    void onMouseWheel(float x, float y) override
    {

        (void)x;
        float yOffset = y;

        // Code for modify camera near plane with orthongonal projection in images space
        float tmpNear = nearPlane + static_cast<float>(yOffset) * 0.75f;

        if (tmpNear <  1.0f)
            nearPlane = 1.0f;
        else if (tmpNear >  radius - 0.5f)
            nearPlane = radius - 0.5f;
        else
            nearPlane = tmpNear;


        //(void)x; (void)y;
    }

    /// -----------------------------
    /// Initialize Renderer, Sensors, and Image Streamers
    /// -----------------------------
    bool onInitialize() override
    {
        // -----------------------------------------
        // Initialize DriveWorks context and SAL
        // -----------------------------------------
        {
            // initialize logger to print verbose message on console in color
            dwLogger_initialize(getConsoleLoggerCallback(true));
            dwLogger_setLogLevel(DW_LOG_VERBOSE);

            // initialize SDK context, using data folder
            dwContextParameters sdkParams = {};
            sdkParams.dataPath = DataPath::get_cstr();

            #ifdef VIBRANTE
            sdkParams.eglDisplay = getEGLDisplay();
            #endif

            dwInitialize(&context, DW_VERSION, &sdkParams);
            dwSAL_initialize(&sal, context);
        }
        // -----------------------------
        // initialize sensors
        // -----------------------------
        {
            initializeSensor(m_args);
        }
        // -----------------------------
        // Initialize Renderer
        // -----------------------------
        {
            initializeInputDefaults();
            initializeRenderer();
        }
        // -----------------------------
        // Start Sensors
        // -----------------------------
        //dwSensor_start(camera);
        gRun = (dwSensor_start(canSensor) == DW_SUCCESS);
/*
        // Initialize radarDynamics, if enabled
        gRunDynamics = (m_args.get("enable-dynamics") == "true");
        if (gRunDynamics) {
            dwStatus ret = radarDynamics_initialize(context, sal, gRadarSensor, m_args);
            if (ret == DW_SUCCESS)
                dynamicsThread = std::thread(radarDynamics_process, &gRun);
        }
*/
        return true;
    }


    ///------------------------------------------------------------------------------
    /// When user requested a reset we playback the video from beginning
    ///------------------------------------------------------------------------------
    void onReset() override
    {
        dwSensor_reset(canSensor);
    }

    ///------------------------------------------------------------------------------
    /// Release acquired memory
    ///------------------------------------------------------------------------------
    void onRelease() override
    {
	    {
	        gRun = false;

	        if (dynamicsThread.joinable())
	            dynamicsThread.join();

	        if (gRunDynamics)
	            radarDynamics_stop();
	        dwSensor_stop(canSensor);
	        dwSAL_releaseSensor(&canSensor);

            // release used objects in correct order
            for (size_t i = 0; i < DW_RADAR_RETURN_TYPE_COUNT; i++) {
                for (size_t j = 0; j < DW_RADAR_RANGE_COUNT; j++) {
                    dwRenderBuffer_release(&gPointCloud[i][j]);
                    delete[] gPointCloudBuffer[i][j];
                }
            }

            if (gPointCloudAsLines != DW_NULL_HANDLE)
            {
                dwRenderBuffer_release(&gPointCloudAsLines);
            }

	        dwRenderBuffer_release(&gGroundPlane);
	        dwRenderBuffer_release(&gGroundCircles);
	        dwRenderBuffer_release(&gWorldSpaceAxes);
	        dwRenderer_release(&renderer);
	    }
	    dwSAL_release(&sal);
	    dwRelease(&context);
	    dwLogger_release();
    }


    ///------------------------------------------------------------------------------
    /// Change renderer properties when main rendering window is resized
    ///------------------------------------------------------------------------------
    void onResizeWindow(int width, int height) override
    {
        dwRect rect;
        rect.width  = width;
        rect.height = height;
        rect.x      = 0;
        rect.y      = 0;
        dwRenderer_setRect(rect, renderer);
    }


    ///------------------------------------------------------------------------------
    /// Main processing of the sample
    ///     - Render
    ///     - Process applciation things
    ///------------------------------------------------------------------------------
    void onProcess() override
    {
        //std::cout << "in onProcess" << std::endl;
        // Render here
        if (renderReady){
            renderFrame();
            renderReady = false;
        }
        //std::cout << "passed renderFrame" << std::endl;
        // Process stuff
        computeSpin();
        //std::cout << "passed computeSpin" << std::endl;
    }

protected:
    //#######################################################################################
    // Initialize Renderer
    //#######################################################################################
    void initializeRenderer()
    {
        CHECK_DW_ERROR_MSG(dwRenderer_initialize(&renderer, context),
                           "Cannot initialize Renderer, maybe no GL context available?");

        // Set some renderer defaults
        gRect.width = getWindowWidth();
        gRect.height = getWindowHeight();
        gRect.x = 0;
        gRect.y = 0;

        dwRenderer_setRect(gRect, renderer);

        float32_t rasterTransform[9];
        rasterTransform[0] = 1.0f;
        rasterTransform[3] = 0.0f;
        rasterTransform[6] = 0.0f;

        rasterTransform[1] = 0.0f;
        rasterTransform[4] = 1.0f;
        rasterTransform[7] = 0.0f;

        rasterTransform[2] = 0.0f;
        rasterTransform[5] = 0.0f;
        rasterTransform[8] = 1.0f;

        dwRenderer_set2DTransform(rasterTransform, renderer);
        dwRenderer_setPointSize(6.0f, renderer);
        dwRenderer_setColor(DW_RENDERER_COLOR_RED, renderer);
        dwRenderer_setFont(DW_RENDER_FONT_VERDANA_16, renderer);

        // Ground plane grid
        constructGrid();
        constructConcentricCircles();

        // Wordld space axis
        constructWorldSpaceAxes();

        // Point cloud
        dwRenderBufferVertexLayout layout;
        layout.posSemantic = DW_RENDER_SEMANTIC_POS_XYZ;
        layout.posFormat   = DW_RENDER_FORMAT_R32G32B32A32_FLOAT;

        layout.colSemantic = DW_RENDER_SEMANTIC_COL_NULL;
        layout.colFormat   = DW_RENDER_FORMAT_NULL;
        layout.texSemantic = DW_RENDER_SEMANTIC_TEX_NULL;
        layout.texFormat   = DW_RENDER_FORMAT_NULL;

        // Initialize Point Clouds
        uint32_t maximumPoints = 10; //gRadarProperties.maxReturnsPerScan;
        size_t maxPointStride = std::max(sizeof(dwRadarDetection), sizeof(dwRadarDetection));
        maxPointStride = std::max(maxPointStride, sizeof(dwRadarStatus));

        for (size_t i = 0; i < DW_RADAR_RETURN_TYPE_COUNT; i++) {
            // Tracks are a bit special
            if (i == DW_RADAR_RETURN_TYPE_TRACK)
                continue;

            for (size_t j = 0; j < DW_RADAR_RANGE_COUNT; j++) {
                dwRenderBuffer_initialize(
                    &gPointCloud[i][j], layout, DW_RENDER_PRIM_POINTLIST,
                    (2 * maximumPoints), context);

                gPointCloudBuffer[i][j] = new char[maximumPoints * maxPointStride];
            }
        }

        // Initialize point cloud for tracks
        layout.colSemantic = DW_RENDER_SEMANTIC_COL_RGB;
        layout.colFormat = DW_RENDER_FORMAT_R32G32B32_FLOAT;
        for (size_t i = 0; i < DW_RADAR_RANGE_COUNT; i++) {
            dwRenderBuffer_initialize(
                &gPointCloud[DW_RADAR_RETURN_TYPE_TRACK][i], layout,
                DW_RENDER_PRIM_POINTLIST, (2 * maximumPoints), context);

            gPointCloudBuffer[DW_RADAR_RETURN_TYPE_TRACK][i] =
                new char[maximumPoints * maxPointStride];
        }

        // Line list with 2 points per line and 3 lines per input point
        dwRenderBuffer_initialize(&gPointCloudAsLines, layout,
            DW_RENDER_PRIM_LINELIST, (2 * maximumPoints), context);
    }

    //#######################################################################################
    // Initialize Sensor
    //#######################################################################################
    void initializeSensor(ProgramArguments &arguments)
    {
        // create Radar interface
        canSensor = DW_NULL_HANDLE;
        {
            dwSensorParams params;

            std::string parameterString;
            std::string protocolString;

            if (strcmp(arguments.get("driver").c_str(), "") != 0) {
                protocolString = arguments.get("driver");

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
            if( dwSAL_createSensor(&canSensor, params, sal) != DW_SUCCESS)
            {
                std::cout << "Cannot create sensor "
                      << params.protocol << " with " << params.parameters << std::endl;

             
/*                // Get Radar properties
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
*/                throw std::runtime_error("Sensor Initialization Failed");
            }
        }
    }

    //#######################################################################################
    // Initialize World Grid
    //#######################################################################################
    void constructGrid()
    {
        // World grid
        int gridResolutionWidth  = static_cast<int>(WORLD_GRID_SIZE_IN_METERS_WIDTH / WORLD_GRID_RES_IN_METERS);
        int gridResolutionHeight = static_cast<int>(WORLD_GRID_SIZE_IN_METERS_HEIGHT / WORLD_GRID_RES_IN_METERS);

        // Rendering data
        dwRenderBufferVertexLayout layout;
        layout.posSemantic = DW_RENDER_SEMANTIC_POS_XYZ;
        layout.posFormat = DW_RENDER_FORMAT_R32G32B32_FLOAT;
        layout.colSemantic = DW_RENDER_SEMANTIC_COL_NULL;
        layout.colFormat = DW_RENDER_FORMAT_NULL;
        layout.texSemantic = DW_RENDER_SEMANTIC_TEX_NULL;
        layout.texFormat = DW_RENDER_FORMAT_NULL;

        dwRenderBuffer_initialize(&gGroundPlane, layout, DW_RENDER_PRIM_LINELIST,
                                  (gridResolutionWidth + 1) + (gridResolutionHeight + 1),
                                  context);

        // update the data
        float32_t *map;
        uint32_t maxVerts, stride;

        dwRenderBuffer_map(&map, &maxVerts, &stride, gGroundPlane);

        int nVertices = 0;
        float x, y;

        // Horizontal lines (parallel to OY, blue axe during rendering)
        x = -0.5f * WORLD_GRID_SIZE_IN_METERS_WIDTH;
        for (int i = 0; i <= gridResolutionWidth; ++i) {
            y = -0.5f * WORLD_GRID_SIZE_IN_METERS_HEIGHT;

            map[stride * nVertices + 0] = x;
            map[stride * nVertices + 1] = y;
            map[stride * nVertices + 2] = WORLD_Z;
            nVertices++;

            y = 0.5f * WORLD_GRID_SIZE_IN_METERS_HEIGHT;
            map[stride * nVertices + 0] = x;
            map[stride * nVertices + 1] = y;
            map[stride * nVertices + 2] = WORLD_Z;

            nVertices++;
            x = x + WORLD_GRID_RES_IN_METERS;
        }

        // Vertical lines (parallel to OX, red axe during rendering
        y = -0.5f * WORLD_GRID_SIZE_IN_METERS_HEIGHT;
        for (int i = 0; i <= gridResolutionHeight; ++i) {
            x = -0.5f * WORLD_GRID_SIZE_IN_METERS_WIDTH;

            map[stride * nVertices + 0] = x;
            map[stride * nVertices + 1] = y;
            map[stride * nVertices + 2] = WORLD_Z;
            nVertices++;

            x = 0.5f * WORLD_GRID_SIZE_IN_METERS_WIDTH;
            map[stride * nVertices + 0] = x;
            map[stride * nVertices + 1] = y;
            map[stride * nVertices + 2] = WORLD_Z;

            nVertices++;
            y = y + WORLD_GRID_RES_IN_METERS;
        }

        dwRenderBuffer_unmap(maxVerts, gGroundPlane);
    }

    void constructConcentricCircles()
    {
        const float WORLD_CIRCLE_RADIUS_START = 0.0f;
        const float WORLD_CIRCLE_RADIUS_END = 50.0f;
        const float WORLD_CIRCLE_LINE_SEGMENTS_PER_METER = 1.0f;

        // Evaluated params
        const size_t numberOfCircles = int((WORLD_CIRCLE_RADIUS_END - WORLD_CIRCLE_RADIUS_START) / WORLD_CIRCLE_DR_IN_METERS + 1);

        std::vector<size_t> numberOfPointsPerCircle;
        numberOfPointsPerCircle.resize(numberOfCircles);

        size_t totalNumberOfPoints = 0;
        for (size_t i = 0; i < numberOfCircles; ++i)
        {
            float r = WORLD_CIRCLE_RADIUS_START + i * WORLD_CIRCLE_DR_IN_METERS;
            numberOfPointsPerCircle[i] = size_t(2 * (2 * DEG2RAD(180.0f) * r) * WORLD_CIRCLE_LINE_SEGMENTS_PER_METER);
            totalNumberOfPoints += numberOfPointsPerCircle[i];
        }

        // Rendering data
        dwRenderBufferVertexLayout layout;
        layout.posSemantic = DW_RENDER_SEMANTIC_POS_XYZ;
        layout.posFormat = DW_RENDER_FORMAT_R32G32B32_FLOAT;
        layout.colSemantic = DW_RENDER_SEMANTIC_COL_NULL;
        layout.colFormat = DW_RENDER_FORMAT_NULL;
        layout.texSemantic = DW_RENDER_SEMANTIC_TEX_NULL;
        layout.texFormat = DW_RENDER_FORMAT_NULL;

        //dwRenderBuffer_initialize(&gGroundCircles, layout, DW_RENDER_PRIM_LINELIST, 2*totalNumberOfPoints/2, context);
        dwRenderBuffer_initialize(&gGroundCircles, layout, DW_RENDER_PRIM_LINELIST, totalNumberOfPoints, context);


        // update the data
        float32_t *map;
        uint32_t maxVerts, stride;

        dwRenderBuffer_map(&map, &maxVerts, &stride, gGroundCircles);

        int nVertices = 0;

        for (size_t i = 0; i < numberOfCircles; ++i)
        {
            float r = WORLD_CIRCLE_RADIUS_START + i * WORLD_CIRCLE_DR_IN_METERS;

            for (size_t p = 0; p < numberOfPointsPerCircle[i]; ++p)
            {
                float theta_1 = DEG2RAD(360.0f) / numberOfPointsPerCircle[i]  * float(p);
                float theta_2 = DEG2RAD(360.0f) / numberOfPointsPerCircle[i]  * float(p + 1);

                float x1 = r * cos(theta_1);
                float y1 = r * sin(theta_1);

                float x2 = r * cos(theta_2);
                float y2 = r * sin(theta_2);

                map[stride * nVertices + 0] = x1;
                map[stride * nVertices + 1] = y1;
                map[stride * nVertices + 2] = WORLD_Z;
                nVertices++;

                map[stride * nVertices + 0] = x2;
                map[stride * nVertices + 1] = y2;
                map[stride * nVertices + 2] = WORLD_Z;

                nVertices++;
            }
        }

        dwRenderBuffer_unmap(maxVerts, gGroundCircles);
    }

    void constructWorldSpaceAxes()
    {
        // Rendering data
        dwRenderBufferVertexLayout layout;
        layout.posSemantic = DW_RENDER_SEMANTIC_POS_XYZ;
        layout.posFormat   = DW_RENDER_FORMAT_R32G32B32_FLOAT;
        layout.colSemantic = DW_RENDER_SEMANTIC_COL_RGB;
        layout.colFormat   = DW_RENDER_FORMAT_R32G32B32_FLOAT;
        layout.texSemantic = DW_RENDER_SEMANTIC_TEX_NULL;
        layout.texFormat   = DW_RENDER_FORMAT_NULL;

        dwRenderBuffer_initialize(&gWorldSpaceAxes, layout, DW_RENDER_PRIM_LINELIST,
                                  2 /*two points per line*/ * 1 /*3 lines per one axe*/ * 3 /*number of axes*/, context);

        // update the data
        float32_t *map;
        uint32_t maxVerts, stride;
        dwRenderBuffer_map(&map, &maxVerts, &stride, gWorldSpaceAxes);

        int nVertices = 0;
        const float AXE_LENGTH = WORLD_GRID_SIZE_IN_METERS_WIDTH / 2.0 * 1.10;

        // Red X-axe
        map[stride * nVertices + 0] = 0;
        map[stride * nVertices + 1] = 0;
        map[stride * nVertices + 2] = WORLD_Z;
        map[stride * nVertices + 3] = 1.0;
        map[stride * nVertices + 4] = 0.0;
        map[stride * nVertices + 5] = 0.0;
        nVertices++;
        map[stride * nVertices + 0] = AXE_LENGTH;
        map[stride * nVertices + 1] = 0;
        map[stride * nVertices + 2] = WORLD_Z;
        map[stride * nVertices + 3] = 1.0;
        map[stride * nVertices + 4] = 0.0;
        map[stride * nVertices + 5] = 0.0;
        nVertices++;

        // Blue Y-axe
        map[stride * nVertices + 0] = 0;
        map[stride * nVertices + 1] = 0;
        map[stride * nVertices + 2] = WORLD_Z;
        map[stride * nVertices + 3] = 0.0;
        map[stride * nVertices + 4] = 0.0;
        map[stride * nVertices + 5] = 1.0;
        nVertices++;
        map[stride * nVertices + 0] = 0;
        map[stride * nVertices + 1] = AXE_LENGTH;
        map[stride * nVertices + 2] = WORLD_Z;
        map[stride * nVertices + 3] = 0.0;
        map[stride * nVertices + 4] = 0.0;
        map[stride * nVertices + 5] = 1.0;
        nVertices++;

        // Green Z-axe
        map[stride * nVertices + 0] = 0;
        map[stride * nVertices + 1] = 0;
        map[stride * nVertices + 2] = WORLD_Z;
        map[stride * nVertices + 3] = 0.0;
        map[stride * nVertices + 4] = 1.0;
        map[stride * nVertices + 5] = 0.0;
        nVertices++;
        map[stride * nVertices + 0] = 0;
        map[stride * nVertices + 1] = 0;
        map[stride * nVertices + 2] = WORLD_Z + AXE_LENGTH;
        map[stride * nVertices + 3] = 0.0;
        map[stride * nVertices + 4] = 1.0;
        map[stride * nVertices + 5] = 0.0;
        nVertices++;


        dwRenderBuffer_unmap(maxVerts, gWorldSpaceAxes);
    }

protected:

    void renderFrame()
    {
        std::cout << "attempting render" << std::endl;
        glDepthFunc(GL_LESS);

        glClearColor(0.0, 0.0, 0.0, 0.0);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // 3D rendering
        lookAt(modelview, eye, center, up);
        ortho(projection, fovRads, 1.0f * gRect.width / gRect.height, nearPlane, 1000.0f);

        dwRenderer_setModelView(modelview, renderer);
        dwRenderer_setProjection(projection, renderer);

        // Render grid
        if (gShowGrid)
        {
            dwRenderer_setColor(DW_RENDERER_COLOR_DARKGREY, renderer);
            dwRenderer_renderBuffer(gGroundPlane, renderer);
            glClear(GL_DEPTH_BUFFER_BIT);
            dwRenderer_setColor(DW_RENDERER_COLOR_GREEN, renderer);
            dwRenderer_renderBuffer(gGroundCircles, renderer);
            glClear(GL_DEPTH_BUFFER_BIT);
        }

        // Render worldspace frame
        {
            dwRenderer_renderBuffer(gWorldSpaceAxes, renderer);
            glClear(GL_DEPTH_BUFFER_BIT);
        }

        dwRenderer_setLineWidth(3.0f, renderer);
        dwRenderer_setColor(DW_RENDERER_COLOR_RED, renderer);

        if (gRenderClusterLines)
            dwRenderer_renderBuffer(gPointCloudAsLines, renderer);

        dwRenderer_setLineWidth(1.0f, renderer);
        std::cout<< "about to render point cloud" << std::endl;
        dwRenderer_setColor(DW_RENDERER_COLOR_RED, renderer);
//        if (gRenderDetections) {
//            for (size_t i = 0; i < DW_RADAR_RANGE_COUNT; i++) {
//        std::cout << "pointcloud: " << gPointCloud[DW_RADAR_RETURN_TYPE_DETECTION][2]
        dwRenderer_renderBuffer(
                    gPointCloud[DW_RADAR_RETURN_TYPE_DETECTION][2], renderer);
//            }
//        }
          std::cout << "should have rendered point cloud" << std::endl;
/*
        dwRenderer_setColor(DW_RENDERER_COLOR_BLUE , renderer);
        if (gRenderTracks) {
            for (size_t i = 0; i < DW_RADAR_RANGE_COUNT; i++) {
                dwRenderer_renderBuffer(
                    gPointCloud[DW_RADAR_RETURN_TYPE_TRACK][i], renderer);
            }
        }
*/
        // Overlay text
        if (gShowText)
        {
            dwRenderer_setColor(DW_RENDERER_COLOR_WHITE, renderer);
            dwRenderer_renderText(20, 200, gMessage1.c_str(), renderer);
            dwRenderer_renderText(20, 180, gMessage2.c_str(), renderer);
            dwRenderer_renderText(20, 160,  gMessage3.c_str(), renderer);
            dwRenderer_renderText(20, 140,  gMessage4.c_str(), renderer);
            dwRenderer_renderText(20, 120,  gMessage5.c_str(), renderer);

            dwRenderer_setColor(DW_RENDERER_COLOR_WHITE, renderer);
            dwRenderer_renderText(20, 100,  gMessage6.c_str(), renderer);

            dwRenderer_setColor(DW_RENDERER_COLOR_ORANGE, renderer);
            dwRenderer_renderText(20, 80,   gMessage7.c_str(), renderer);

            dwRenderer_setColor(DW_RENDERER_COLOR_RED, renderer);
            dwRenderer_renderText(20, 60,   gMessage8.c_str(), renderer);

            dwRenderer_setColor(DW_RENDERER_COLOR_GREEN, renderer);
            dwRenderer_renderText(20, 40,   gMessage9.c_str(), renderer);

            dwRenderer_setColor(DW_RENDERER_COLOR_WHITE, renderer);
            dwRenderer_renderText(20, 20,    gMessage10.c_str(), renderer);

            dwRenderer_setColor(DW_RENDERER_COLOR_WHITE, renderer);
            dwRenderer_renderText(20, 0,    gMessage11.c_str(), renderer);
        }
    }

    double updateFrequency()
    {
        return 20;
        //return gRadarProperties.scansPerSecond * gFreqMultiplier;
    }

//    dwRadarDetection parseCANtarget(dwCANMessage msg)
    float32_t * parseCANtarget(dwCANMessage msg)
    {
        //dwRadarDetection detection;
        static float32_t decoded[13] = {0};

        if(msg.size > 0){
/*
            detection.radius = (int16_t)( (msg.data[2] << 8) + msg.data[3]) / 100.0; 
            detection.azimuth = (int16_t)( (msg.data[6] << 8) + msg.data[7]) / 100.0 * -1;
            detection.radialVelocity = (int16_t)( (msg.data[4] << 8) + msg.data[5]) / 100.0;
            detection.rcs = msg.data[1];
*/
            decoded[7] = (float32_t)(int16_t)( (msg.data[2] << 8) + msg.data[3]) / 100.0;
            decoded[6] = (float32_t)(int16_t)( (msg.data[6] << 8) + msg.data[7]) / 100.0 * -1;
            decoded[8] = (float32_t)(int16_t)( (msg.data[4] << 8) + msg.data[5]) / 100.0;
            decoded[10] = (float32_t)msg.data[1];
            decoded[0] = 1;
            decoded[1] = 2;
            decoded[2] = 0;
            decoded[3] = 0;
            decoded[4] = 0;
            decoded[5] = 0;
            decoded[9] = 0;
            decoded[11] = 0;
            decoded[12] = 1;
        }

        return decoded;

    }

    float32_t * collectScan()
    {
        static float32_t dataArray[10] = {0};
        dwCANMessage msg;

        int targetCount = 0;
        float32_t* received;

        while(!renderReady)
        {
            dwSensorCAN_readMessage(&msg, 100000, canSensor);
            //This is a header frame for Kanza-77
            if( msg.id == 0x480 ) {//43e
                readyForTargets = true;
                std::cout << "header id: " << std::hex << (int)msg.id << std::endl << std::dec;
            //    continue;
            //This is a footer frame for Kanza-77
            } else if( (msg.id == 0x481) && (readyForTargets) ) {//43f
                //this is where you direct the output of the detections vector.          
                readyForTargets = false;
                std::cout << "footer id: " << std::hex << (int)msg.id << std::endl << std::dec;
                renderReady = true;
            //    continue;
            //Everything else is either a target or a radar ACK
            } else if( readyForTargets ){
                std::cout << "target id: " << std::hex << (int)msg.id << std::endl << std::dec;
                received = parseCANtarget(msg);
                std::cout << "SNR: "      << received[10] << std::endl;
                std::cout << "distance: " << received[7] << std::endl;
                std::cout << "velocity: " << received[8] << std::endl;
                std::cout << "angle: "    << received[6] << std::endl;

                dataArray[targetCount] = *received;

                targetCount ++;
            }
        }
        std::cout << std::endl;

        return dataArray;

    }

    void computeSpin()
    {
        float32_t *packetArray;

        packetArray = collectScan();

        std::cout << "passed collectScan" << std::endl;

//        static uint32_t packetCount = 0;
        static std::chrono::system_clock::time_point t_start = std::chrono::high_resolution_clock::now();
        static std::chrono::system_clock::time_point t_end;

        dwTime_t timey;
        const dwRadarScanType &type{.returnType = DW_RADAR_RETURN_TYPE_DETECTION, .range = DW_RADAR_RANGE_LONG};

/*
        std::cout << msg.timestamp_us << " " << msg.id << std::endl;
        if (status != DW_SUCCESS) {
            std::cout << " ERROR " << dwGetStatusName(status);
        }
*/
/*
        // Allow pausing for recoded replay
        if(gRecordedRadar && gPause)
            return;

        // For recorded data thottling check how long to a full sping and match to the radar frequency
        if(gRecordedRadar) {
            t_end = std::chrono::high_resolution_clock::now();
            double duration  = std::chrono::duration<double, std::milli>(t_end-t_start).count();
            double sleepTime = 1000.0 / updateFrequency() - duration;
            // This ensures proper behavior and quick restart in cause of pauses.
            sleepTime = (sleepTime * 1000 > 0) ? sleepTime : 100;

            usleep(sleepTime * 1000);
        }
*/
        // Empty the queue and append all points withing the same spin.
        // Update render structures only when a full spin is done.
        dwStatus status = DW_SUCCESS;

        t_start = std::chrono::high_resolution_clock::now();

        size_t accumulatedPoints[DW_RADAR_RETURN_TYPE_COUNT][DW_RADAR_RANGE_COUNT]{};

        // ToDo: this is just temporary logic until we return full scans (skadle)
        for (size_t i = 0; i < 10; i++) {
//            status = dwSensorRadar_readScan(&nextPacket,
//                {}, 100000, gRadarSensor);

//            float32_t * newArray = &packetArray[i];
            const dwRadarScan nextPacket = 
            { 
                1,
                timey,
                timey,
                type,
                1,
                static_cast<void*>(&packetArray[i])
            };

        std::cout << " made nextPacket " << std::endl;
            std::cout << "looping through nextPacket, index: " << i << std::endl;
            if (status == DW_SUCCESS) {
//                const dwRadarScanType &type = nextPacket->scanType;
                //type.returnType = DW_RADAR_RETURN_TYPE_DETECTION;
                //type.range = DW_RADAR_RANGE_LONG;


//              switch (type.returnType)
//                {
//                    case DW_RADAR_RETURN_TYPE_DETECTION:
                        memcpy(gPointCloudBuffer[type.returnType][type.range] +
                            accumulatedPoints[type.returnType][type.range] * sizeof(dwRadarDetection),
                            nextPacket.data,
                            nextPacket.numReturns * sizeof(dwRadarDetection));
//                        break;
/*                    case DW_RADAR_RETURN_TYPE_TRACK:
                        memcpy(gPointCloudBuffer[type.returnType][type.range] +
                            accumulatedPoints[type.returnType][type.range] * sizeof(dwRadarTrack),
                            nextPacket->data,
                            nextPacket->numReturns * sizeof(dwRadarTrack));
                        break;
                    case DW_RADAR_RETURN_TYPE_STATUS:
                        break;
                    case DW_RADAR_RETURN_TYPE_COUNT:
                    default:
                        std::cout << "RadarReplay: Invalid point type received"<<std::endl;
                        break;
                }
*/
                accumulatedPoints[type.returnType][type.range] += nextPacket.numReturns;

//                gMessage1 = "Host timestamp " + toStr(nextPacket->hostTimestamp/1000000ULL) + "(sec) / "
//                                              + toStr(nextPacket->hostTimestamp) + "(millisecs)";
//                gMessage2 = "Sensor timestamp " + toStr(nextPacket->sensorTimestamp/1000000ULL) + "(sec) / "
//                                              + toStr(nextPacket->sensorTimestamp) + "(millisecs)";

//                dwSensorRadar_returnScan(nextPacket, gRadarSensor);
//                packetCount++;
            }
        }


        std::cout << "passed nextPacket loop" << std::endl;

        float32_t *map;
        uint32_t maxVerts, stride;

        for (size_t i = 0; i < 1; i++) {
            for (size_t j = 2; j < 3; j++) {
                //if (!gRadarProperties.supportedScanTypes[i][j])
                //    continue;
                std::cout << "in the not a loop loop, point in array: " << accumulatedPoints[i][j] << std::endl;
                // Map to the point cloud
                dwRenderBuffer_map(&map, &maxVerts, &stride, gPointCloud[i][j]);

                for (size_t k = 0; k < accumulatedPoints[i][j]; k++) {
                    std::cout << " looping over accumulated points, index: " << k << std::endl;
                    if (static_cast<dwRadarReturnType>(i) == DW_RADAR_RETURN_TYPE_DETECTION) {
                        std::cout << "in the detection" << std::endl;
                        dwRadarDetection* updatePoint =
                            reinterpret_cast<dwRadarDetection*>
                                (gPointCloudBuffer[i][j] +
                                    k * sizeof(dwRadarDetection));

                        map[k * stride + 0] = updatePoint->x;
                        map[k * stride + 1] = updatePoint->y;
                        map[k * stride + 2] = WORLD_Z;
                        map[k * stride + 3] = 0.0f;

                        map[2 * k * stride + 4] = 0.5f;
                        map[2 * k * stride + 5] = 0.5f;
                        map[2 * k * stride + 6] = 0.5f;
                        map[2 * k * stride + 7] = 1.0f;

                    } else if (static_cast<dwRadarReturnType>(i) == DW_RADAR_RETURN_TYPE_TRACK) {
                        dwRadarTrack* updatePoint =
                            reinterpret_cast<dwRadarTrack*>
                                (gPointCloudBuffer[i][j] +
                                    k * sizeof(dwRadarTrack));

                        map[k * stride + 0] = updatePoint->x;
                        map[k * stride + 1] = updatePoint->y;
                        map[k * stride + 2] = WORLD_Z;
                        map[k * stride + 3] = 1.0f;

                        switch (updatePoint->dynamicState) {
                            case DW_RADAR_DYNAMIC_STATE_STATIONARY:
                                map[k * stride + 4] = 0.1f;
                                map[k * stride + 5] = 0.9f;
                                map[k * stride + 6] = 0.1f;
                                break;
                            case DW_RADAR_DYNAMIC_STATE_MOVING:
                                map[k * stride + 4] = 0.1f;
                                map[k * stride + 5] = 0.1f;
                                map[k * stride + 6] = 0.9f;
                                break;
                            case DW_RADAR_DYNAMIC_STATE_ONCOMING:
                                map[k * stride + 4] = 0.9f;
                                map[k * stride + 5] = 0.9f;
                                map[k * stride + 6] = 0.1f;
                                break;
                            default:
                                map[k * stride + 4] = 1.0f;
                                map[k * stride + 5] = 1.0f;
                                map[k * stride + 6] = 1.0f;
                                break;
                        }
                    }
                }
                std::cout << "pre unmap" << std::endl;
                dwRenderBuffer_unmap(accumulatedPoints[i][j], gPointCloud[i][j]);
                std::cout << "post unmap" << std::endl;
            }
        }
/*
        gMessage3 = "Packets                     " + toStr(packetCount);
        gMessage4 = "Application frequency for read Radar Scans (Hz)   " + toStr(updateFrequency());
        gMessage6 = "Step in the rect grid: " + toStr(WORLD_GRID_RES_IN_METERS) + "m / radial grid: " + toStr(WORLD_CIRCLE_DR_IN_METERS) + "m";
        gMessage7 = "Worldspace axes: Red-OX, Blue-OY, Green - OZ.)";
        gMessage8 = "Velocity vector is marked with red if speed <= " + toStr(WORLD_VELOCITY_LOW_KM_PER_HOUR) + " KM/H";
        gMessage9 = "Velocity vector is marked with green if speed >= " + toStr(WORLD_VELOCITY_HIGH_KM_PER_HOUR) + " KM/H";
        gMessage10 = "Press 'ESC' - to exit / 'R'-reset / 'G'- show/hide grid / 'F1' - show/hide text messages / '+-' - increase/decrease reading scan rate ";

        gMessage11 = "/ 'Toggle 0: Detections, 1: Clusters, 2: Tracks, 4: Cluster Lines";
*/
        // Grab properties, in case they were changed while running
//        dwSensorRadar_getProperties(&gRadarProperties, gRadarSensor);
/*
        // For recorded data, start over at the end of the file
        if (status == DW_END_OF_STREAM) {
            dwSensor_reset(gRadarSensor);
            packetCount = 0;
        }
*/
    }
};

//------------------------------------------------------------------------------
int main(int argc, const char **argv)
{
    std::cout << "updated version check 7" << std::endl;

    std::string dynamicsParams;
    dynamicsParams = "can-driver=can.socket"
                     ",can-params=device=can0"
                     ",rig-configuration=wwdc_rig.xml"
                     ",radar-name=FL"
                     ",isReversed=false"
                     ",radome-damping=0.0";

    ProgramArguments arguments(argc, argv,
    {
        ProgramArguments::Option_t("driver", "can.socket"),
        ProgramArguments::Option_t("params", "device=can0"),
        ProgramArguments::Option_t("enable-dynamics", "false"),
        ProgramArguments::Option_t("dynamics-params", dynamicsParams.c_str()),
    },
    "Radar replay sample which playback .bin video streams in a GL window.");

    // initialize and start a window application
    RadarReplay app(arguments);

    app.initializeWindow("Radarded replay", 1024, 800, arguments.enabled("offscreen"));

    return app.run();
}
