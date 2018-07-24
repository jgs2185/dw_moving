

#include <errno.h>
#include <signal.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <fcntl.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

#include <linux/can.h>
#include <linux/can/raw.h>

//#include "target.cpp"
#include "radar.cpp"

#include <dw/sensors/radar/Radar.h>
#include <dw/sensors/plugins/radar/RadarDecoder.h>

Radar my_radar_test("k77",0.0,0.0,0.0);

dwStatus _dwRadarDecoder_initialize(){
//once we can actually make a radar object, should figure out how to make
//this work in that context

    my_radar_test.init();

    my_radar_test.activate();


    dwStatus ret = DW_SUCCESS;

    return ret;
}


dwStatus _dwRadarDecoder_release(){


    my_radar_test.deactivate();    

    dwStatus ret = DW_SUCCESS;

    return ret;

}


dwStatus _dwRadarDecoder_getConstants(_dwRadarDecoder_constants *constants){

    constants->properties.isDecodingOn = 0;
    constants->properties.inputPacketsPerSecond = 0; //TODO: what should this
						     //be if the radar doesn't take it as input
    constants->properties.maxReturnsPerScan = 128;
    constants->properties.numScanTypes = 2;
    constants->properties.scansPerSecond = 20; //TODO find out what this really is
    constants->properties.packetsPerScan = 62;
    for(int typecount = 0; typecount < DW_RADAR_RETURN_TYPE_COUNT; typecount ++){
        for(int rangecount = 0; rangecount < DW_RADAR_RANGE_COUNT; rangecount ++){
            constants->properties.supportedScanTypes[typecount][rangecount]=0;
        }
    }

    constants->properties.supportedScanTypes[0][0] = 1;
    constants->properties.supportedScanTypes[0][2] = 1;

    
    constants->headerSize = 8;
    constants->maxPayloadSize = 536;//each scan has 8 header byte, 8 footer byte and up to 65 targets at 8 bytes = 16 + 520 = 536
    constants->vehicleStateSize = 0;
    constants->mountSize = 0;

    dwStatus ret = DW_SUCCESS;

    return ret;
}

dwStatus _dwRadarDecoder_decodePacket(dwRadarScan  *output,
                                      const uint8_t *buffer,
                                      const size_t length,
                                      const dwRadarScanType scanType){

    dwStatus ret;

    float32_t decoded[13] = {0};

    if(length == 8 && scanType.returnType  == DW_RADAR_RETURN_TYPE_DETECTION){
        decoded[7] = (float32_t)(int16_t)( (buffer[2] << 8) + buffer[3]) / 100.0;
        decoded[6] = (float32_t)(int16_t)( (buffer[6] << 8) + buffer[7]) / 100.0 * -1;
        decoded[8] = (float32_t)(int16_t)( (buffer[4] << 8) + buffer[5]) / 100.0;
        decoded[10] = (float32_t)buffer[1];
        //int id = buffer[0];  //this one only matters for the tracking return type
        std::cout << std::dec << "scnan data: " << decoded[7]<< std::endl;
        output->data = decoded;
    
        ret = DW_SUCCESS;
    } else {
        ret = DW_FAILURE;
    }

    return ret;

}


dwStatus _dwRadarDecoder_synchronize(const uint8_t *buffer,
                                     const size_t length,
                                     size_t *remaining){

    dwStatus ret;

    //this one will check if it's a header, if it is, return true and update remaining
    //need to frame count get this info put into the header

    //remaining = &sizeof(buffer);

    uint32_t id = (uint32_t)((buffer[0] << 24) + (buffer[1] << 16) + (buffer[2] << 8) + buffer[3]);

    std::cout << id << " "  << length << remaining << std::endl;

    if(id == 1086){
        ret = DW_SUCCESS;
    } else {
        ret = DW_FAILURE;
    }

    return ret;

}


dwStatus _dwRadarDecoder_validatePacket(const uint8_t *buffer,
                                        const size_t length,
                                        dwRadarScanType *scanType){

    //this one will check if ID is valid? should be a checksum
    //byte size 16

    dwStatus ret;

    dwRadarScanType testType = {DW_RADAR_RETURN_TYPE_DETECTION,DW_RADAR_RANGE_SHORT};
 
    scanType->range = testType.range;
    scanType->returnType = testType.returnType;

    std::cout << length << scanType->range << testType.range << std::endl;

    uint32_t id = (uint32_t)((buffer[0] << 24) + (buffer[1] << 16) + (buffer[2] << 8) + buffer[3]);

    if( id >= 1024 && id <= 1085 ){
        
        ret = DW_SUCCESS;
    } else {
        ret = DW_FAILURE;
    }

    return ret;

}


dwBool _dwRadarDecoder_isScanComplete(dwRadarScanType scanType,
                                      const uint8_t **buffer,
                                      size_t *length,
                                      size_t numPackets){
    //this one will check for header and footer
    //will count 'targets' in the buffer return false if last 'target' is a target, true if last 'target' is the footer

    dwBool ret = true;

    return ret;


}


dwStatus _dwRadarDecoder_encodeVehicleState(uint8_t *buffer,
                                            const size_t maxOutputSize,
                                            const dwRadarVehicleState *packet){

    dwStatus ret = DW_SUCCESS;

    return ret;


}


dwStatus _dwRadarDecoder_encodeMountPosition(uint8_t *buffer,
                                             const size_t maxOutputSize,
                                             const dwRadarMountPosition *packet){

    dwStatus ret = DW_SUCCESS;

    return ret;


}

