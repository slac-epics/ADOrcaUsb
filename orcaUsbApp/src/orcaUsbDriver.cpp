/*
 * Area Detector Driver for the Orca USB
 *
 * Author Janos, Vamosi
 * Date   Feb-28-2021
 *
 * Heavily modified: M. Dunning (mdunning) 19-Sep-2024
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <unistd.h>

#include <epicsTime.h>
#include <epicsThread.h>
#include <epicsEvent.h>
#include <epicsString.h>
#include <epicsExit.h>
#include <epicsExport.h>
#include <iocsh.h>
#include <errlog.h>

#include "orcaUsbDriver.h"


namespace {
    const std::string driverName = "OrcaUsbDriver";
}

extern "C" {
//#include "perfMeasure.h"
//#include "edtinc.h"
//int deIntlv_MidTop_Line16(u_short *src, int width, int rows, u_short *dest);
//int pdv_query_serial(PdvDev * pdv_p, char *cmd, char **resp);
//int ErGetTicks(int, unsigned*);
#define LOWER_17_BIT_MASK       (0x0001FFFF)    /* (2^17)-1            */
#define PULSEID(time)           ((time).nsec & LOWER_17_BIT_MASK)
}

// define default image size and offset
// for multiple camera support size should be smaller than max size (2048)
// this limitation is coming from experience with driver usage
#define MIN_X   4
#define MIN_Y   4
#define SIZE_X  2040
#define SIZE_Y  2040

/**
 @def	USE_COPYFRAME
 *
 *0:	dcambuf_lockframe is used to access image
 *		This function gets the pointer of images, so it is necessary to copy the target ROI from this poitner. 
 *      It is possible to calculate the top pointer of each image bundled with the properties related framebundle.
 *
 *1:	dcambuf_copyframe is used to access image
 *		This function sets the pointer of buffer to get the images. 
 *      DCAM copies the target ROI of each image to this pointer. 
 */

// 0: call dcambuf_lockframe to access image, 1: call dcambuf_copyframe to access image
#define USE_COPYFRAME 1


static void dataTaskC(void *drvPvt) {
/**********************************************************************
    Main data task function pointer.
**********************************************************************/
    OrcaUsbDriver *pOrca = (OrcaUsbDriver*) drvPvt;
    pOrca->dataTask();
}


static void exitHookC(void *drvPvt) {
/**********************************************************************
 * Callback function called when IOC is terminated.
 * Deletes created object and frees Orca context.
 *
 * \params[in]: pointer to the OrcaUsb object created in OrcaUsbConfig
**********************************************************************/
    OrcaUsbDriver *pOrca = (OrcaUsbDriver*) drvPvt;
    epicsPrintf("Orca camera %d driver exiting...\n", pOrca->getCamIndex());
    delete pOrca;
}


/**********************************************************************
    OrcaUsb Constructor
**********************************************************************/
/** Constructor for OrcaUsb driver; most parameters are simply passed to ADDriver::ADDriver.
  * After calling the base class constructor this method creates a thread to collect the detector data, 
  * and sets reasonable default values for the parameters defined in this class, asynNDArrayDriver and ADDriver.
  * \param[in]: The name of the asyn port driver to be created.
  * \param[in]: The ID (serial number) of the camera or "0" for the next available camera.
  * \param[in]: The maximum number of NDArray buffers that the NDArrayPool for this driver is 
  *             allowed to allocate. Set this to -1 to allow an unlimited number of buffers.
  * \param[in]: The maximum amount of memory that the NDArrayPool for this driver is 
  *             allowed to allocate. Set this to -1 to allow an unlimited amount of memory.
  * \param[in]: The thread priority for the asyn port driver thread if ASYN_CANBLOCK is set in asynFlags.
  * \param[in]: The stack size for the asyn port driver thread if ASYN_CANBLOCK is set in asynFlags.
  */
OrcaUsbDriver::OrcaUsbDriver(const char *portName, const char* cameraId, int maxBuffers, size_t maxMemory, int priority, int stackSize)
        : ADDriver(portName, 1, NUM_ORCA_DET_PARAMS, maxBuffers, maxMemory,
               0, 0,               /* No interfaces beyond those set in ADDriver.cpp */
               ASYN_CANBLOCK, 1,   /* ASYN_CANBLOCK=1, ASYN_MULTIDEVICE=0, autoConnect=1 */
               priority, stackSize),
        port_name(portName),
        acquire(false),
        exit_loop(false),
        exited(false),
        hdcam(NULL)
{
    char    str_message[128];
    double  tmpfloat64;
    int     tmpint32;
    DCAMERR err;

    // initCounter maintains driver instances: 
    // i.e. how many times OrcaConfig called from st.cmd
    // DCAM-API should be initialized once only on a computer
    if (initCounter == 0) {
        memset(&apiinit, 0, sizeof(apiinit));
        apiinit.size = sizeof(apiinit);

        // initialize DCAM-API
        err = dcamapi_init(&apiinit);

        if (failed(err)) {
            errlogPrintf("Error: initializing API\n");
            return;
        }

        for (int i=0; i<MAX_CAM_NUM; i++) {
            openCameras[i] = 0;
        }

        deviceCount = apiinit.iDeviceCount;
        //epicsPrintf("Found %d device(s).\n", deviceCount);
    }


    // set index of camera accrding to driver instance index
    // even if no camera found for this instance
    // not available cameras will report error when called
    cameraIndex = initCounter;

    // increase counter to maintain number of driver instances
    // (i.e. number of attempts to connect new cameras from EPICS)
    initCounter++;

    cameraAvailable = findCamera(cameraId);

    // set hdcam to -1 as NULL (0) is accepted by dcamdev_getstring()
    // dcamdev_getstring() can be called with the device index too
    // and returns with values related to other camera with index 0
    if (cameraAvailable == 0)
        hdcam = (HDCAM) -1;

    dataEvent = epicsEventMustCreate(epicsEventEmpty);

    // Asyn parameter library
    // Plugins need to call the createParam() function in their constructor 
    // if they have additional parameters beyond those in the asynPortDriver or NDPluginDriver base classes
    createParam(OrcaCameraNameString,               asynParamOctet, &cameraName);
    createParam(OrcaCameraSerialString,             asynParamOctet, &cameraSerial);
    createParam(OrcaCameraFirmwareString,           asynParamOctet, &cameraFirmware);
    createParam(OrcaCameraInfoString,               asynParamOctet, &cameraInfo);
    createParam(OrcaCameraTrgString,                asynParamInt32, &cameraTrg);
    createParam(OrcaCameraTrgPolarityString,        asynParamInt32, &cameraTrgPolarity);
    createParam(OrcaCameraTrgGlobalExposureString,  asynParamInt32, &cameraTrgGlobalExposure);
    createParam(OrcaCameraReadStatString,           asynParamInt32, &cameraReadStat);
    createParam(OrcaCameraStatusString,             asynParamInt32, &cameraStatus);


    setStringParam(ADManufacturer, "Hamamatsu");

    if (getCameraName(str_message) != -1) {
        setStringParam(cameraName, str_message);  
        setStringParam(ADModel, str_message); 
    } else {
        setStringParam(cameraName, "");  
        setStringParam(ADModel, ""); 
    }

    if (getCameraSerial(str_message) != -1) 
        setStringParam(cameraSerial, str_message);
    else
        setStringParam(cameraSerial, "");

    if (getCameraFirmware(str_message) != -1) 
        setStringParam(cameraFirmware, str_message);
    else
        setStringParam(cameraFirmware, "");

    if (getCameraInfo(str_message) != -1)
        setStringParam(cameraInfo, str_message);
    else
        setStringParam(cameraInfo, "");

    setIntegerParam(cameraTrg, 0);         /* Edge Trigger */
    setIntegerParam(cameraTrgPolarity, 0); /* Negative Polarity */

    //setIntegerParam(cameraTrgGlobalExposure, 5); /* Global Reset */

    if (getActualExposure(&tmpfloat64) != -1)
        setDoubleParam(ADAcquireTime, tmpfloat64);
    else
        setDoubleParam(ADAcquireTime, 0);

    // get sensor size X
    if (getEffectiveSizeX(&tmpint32) != -1) {
        // set base class parameter default value        
        maxSizeX = tmpint32;
        setIntegerParam(ADMaxSizeX, maxSizeX);
    } else {
        setIntegerParam(ADMaxSizeX, SIZE_X);
        maxSizeX = SIZE_X;
    }

    // get sensor size Y
    if (getEffectiveSizeY(&tmpint32) != -1) {
        // set base class parameter default value        
        maxSizeY = tmpint32;
        setIntegerParam(ADMaxSizeY, maxSizeY);
    } else {
        setIntegerParam(ADMaxSizeY, SIZE_Y);
        maxSizeY = SIZE_Y;
    }

    // set image size and offset
    // image size should be smaller than max to support multiple cameras
    minX = MIN_X;
    sizeX = SIZE_X;
    minY = MIN_Y;
    sizeY = SIZE_Y;
    
    // set base class parameter default values        
    setIntegerParam(ADMinX, minX);
    setIntegerParam(ADMinY, minY);
    setIntegerParam(ADSizeX, sizeX);
    setIntegerParam(ADSizeY, sizeY);
    setIntegerParam(NDArraySizeX, sizeX);
    setIntegerParam(NDArraySizeY, sizeY);
    setIntegerParam(NDArraySize, sizeX*sizeY*sizeof(epicsUInt16));
    setIntegerParam(NDDataType, NDUInt16);
    setIntegerParam(NDArrayCallbacks, 1);

    //set global exposure to GLOBAL RESET (default is DELAYED)
    setTriggerGlobalExposure(DCAMPROP_TRIGGER_GLOBALEXPOSURE__GLOBALRESET);

    // Set default ROI
    setROI(minX, sizeX, minY, sizeY);

    // register shutdown function for epicsAtExit
    epicsAtExit(exitHookC, this);

    // create new thread (it terminates when funtion ptr returns)
    epicsThreadCreate("OrcaTask",                      /* name */
                       epicsThreadPriorityMedium,      /* priority */
                       stackSize,                      /* stack size */
                       (EPICSTHREADFUNC) dataTaskC,    /* funtion pointer */
                       this);                          /* argument passed */ 
}


OrcaUsbDriver::~OrcaUsbDriver() {
/**********************************************************************
    OrcaUsb Destructor. Called by the exitHookC function when IOC is shut down.
**********************************************************************/
    std::string functionName = "~OrcaUsbDriver";
    this->lock();
    exit_loop = true;
    acquire = false;
    this->unlock();
    epicsEventSignal(dataEvent);
    if (hdcam != NULL) {
        dcamdev_close(hdcam);
        //epicsPrintf("Camera %d closed on exit\n", cameraIndex);
        asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
                "%s::%s: port %s: Camera %d closed\n", 
                driverName.c_str(), functionName.c_str(), port_name.c_str(), cameraIndex);
    }

    while (!exited) {
        epicsThreadSleep(0.2);
    }
}


int OrcaUsbDriver::getCameraName(char *value) {
/**********************************************************************
**********************************************************************/
    std::string functionName = "getCameraName";
    DCAMERR err;
    char data[256];

    if (!value)
        return -1;

    DCAMDEV_STRING param;
    memset( &param, 0, sizeof(param) );
    param.size = sizeof(param);
    param.text = data;
    param.textbytes = sizeof(data);
    param.iString = DCAM_IDSTR_MODEL;

    err = dcamdev_getstring(hdcam, &param);
 
    if (!failed(err)) {
        strcpy(value, data);
        asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
                "%s::%s: %s: Camera name: %s\n",
                driverName.c_str(), functionName.c_str(), port_name.c_str(), value);
    } else {
        printCameraError(cameraIndex, hdcam, err, "dcamprop_getvalue(DCAM_IDSTR_MODEL)\n");
        return -1;
    }

    return 0;
}


int OrcaUsbDriver::getCameraSerial(char *value){
/**********************************************************************
**********************************************************************/
    std::string functionName = "getCameraSerial";
    int  ret = 0;
    DCAMERR err;
    char data[256];

    if (!value)
        return -1;

    DCAMDEV_STRING param;
    memset( &param, 0, sizeof(param) );
    param.size = sizeof(param);
    param.text = data;
    param.textbytes = sizeof(data);
    param.iString = DCAM_IDSTR_CAMERAID;

    err = dcamdev_getstring(hdcam, &param);
    if (!failed(err)) {
        strcpy(value, data);
        asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
                "%s::%s: port %s: Camera serial: %s\n",
                driverName.c_str(), functionName.c_str(), port_name.c_str(), value);
    } else {
        printCameraError(cameraIndex, hdcam, err, "dcamprop_getvalue(DCAM_IDSTR_CAMERAID)\n");
        ret = -1;
    }

    return ret;
}


int OrcaUsbDriver:: getCameraFirmware(char *value) {
/**********************************************************************
**********************************************************************/
    std::string functionName = "getCameraFirmware";
    DCAMERR err;
    char data[256];

    if (!value)
        return -1;

    DCAMDEV_STRING param;
    memset( &param, 0, sizeof(param) );
    param.size = sizeof(param);
    param.text = data;
    param.textbytes = sizeof(data);
    param.iString = DCAM_IDSTR_CAMERAVERSION;

    err = dcamdev_getstring(hdcam, &param);
    if (!failed(err)) {
        strcpy(value, data);
        asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
                "%s::%s: port %s: Camera firmware: %s\n",
                driverName.c_str(), functionName.c_str(), port_name.c_str(), value);
    } else {
        printCameraError(cameraIndex, hdcam, err, "dcamprop_getvalue(DCAM_IDSTR_CAMERAVERSION)\n");
        return -1;
    }

    return 0;
}


int OrcaUsbDriver::getCameraInfo(char *value) {
/**********************************************************************
**********************************************************************/
    std::string functionName = "getCameraInfo";
    DCAMERR err;
    char data[256];

    if (!value)
        return -1;

    DCAMDEV_STRING param;
    memset( &param, 0, sizeof(param) );
    param.size = sizeof(param);
    param.text = data;
    param.textbytes = sizeof(data);
    param.iString = DCAM_IDSTR_CAMERA_SERIESNAME;

    err = dcamdev_getstring(hdcam, &param);
    if (!failed(err)) {
        strcpy(value, data);
        asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
                "%s::%s: port %s: Camera series: %s\n",
                driverName.c_str(), functionName.c_str(), port_name.c_str(), value);
    } else {
        printCameraError(cameraIndex, hdcam, err, "dcamprop_getvalue(DCAM_IDSTR_SERIESNAME)\n");
        return -1;
    }

    return 0;
}


int OrcaUsbDriver::getCameraStatus() {
/**********************************************************************
    A somewhat ugly routine to determine if the camera is connected.
**********************************************************************/
    std::string functionName = "getCameraStatus";
    DCAMERR err;
    char data[16];
    double alive = -1.0, temp = -1.0;

    DCAMDEV_STRING param;
    memset( &param, 0, sizeof(param) );
    param.size = sizeof(param);
    param.text = data;
    param.textbytes = sizeof(data);
    param.iString = DCAM_IDSTR_CAMERAID;

    dcamdev_getstring(hdcam, &param);
    dcamprop_getvalue(hdcam, DCAM_IDPROP_SENSORTEMPERATURE, &temp);
    err = dcamprop_getvalue(hdcam, DCAM_IDPROP_SYSTEM_ALIVE, &alive);

    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
            "%s::%s: port %s: alive=%f, temp=%f, err=0x%x\n", 
            driverName.c_str(), functionName.c_str(), port_name.c_str(), alive, temp, err);

    bool ret = ((int)alive == 2)?1:0;
    return ret;
}


int OrcaUsbDriver::getActualExposure(double *value) {
/**********************************************************************
**********************************************************************/
    int ret = 0;
    DCAMERR err;
    double data;

    if(!value)
        return -1;

    err = dcamprop_getvalue( hdcam, DCAM_IDPROP_EXPOSURETIME, &data );
    if( !failed(err) )
    {
        *value = data;
    }
    else
    {
        printCameraError(cameraIndex, hdcam, err, "dcamprop_getvalue(DCAM_IDPROP_EXPOSURETIME)\n");
        ret = -1;
    }

    return ret;
}


int OrcaUsbDriver::getEffectiveSizeX(int *value) {
/**********************************************************************
**********************************************************************/
    int ret = 0;
    DCAMERR err;
    double data;

    if(!value)
        return -1;

    err = dcamprop_getvalue( hdcam, DCAM_IDPROP_IMAGE_WIDTH, &data );
    if( !failed(err) )
    {
        *value = data;
    }
    else
    {
        printCameraError(cameraIndex, hdcam, err, "dcamprop_getvalue(DCAM_IDPROP_IMAGE_WIDTH)\n");
        ret = -1;
    }

    return ret;
}


int OrcaUsbDriver::getRowBytes(int *value) {
/**********************************************************************
**********************************************************************/
    DCAMERR err;
    double data;

    if (!value) {
        return -1;
    }

    err = dcamprop_getvalue(hdcam, DCAM_IDPROP_IMAGE_ROWBYTES, &data);
    if (!failed(err)) {
        *value = data;
    } else {
        printCameraError(cameraIndex, hdcam, err, "dcamprop_getvalue(DCAM_IDPROP_IMAGE_ROWBYTES)\n");
        return -1;
    }

    return 0;
}


int OrcaUsbDriver::getEffectiveSizeY(int *value) {
/**********************************************************************
**********************************************************************/
    int ret = 0;
    DCAMERR err;
    double data;

    if(!value)
        return -1;

    err = dcamprop_getvalue( hdcam, DCAM_IDPROP_IMAGE_HEIGHT, &data );
    if( !failed(err) )
    {
        *value = data;
    }
    else
    {
        printCameraError(cameraIndex, hdcam, err, "dcamprop_getvalue(DCAM_IDPROP_IMAGE_HEIGHT)\n");
        ret = -1;
    }

    return ret;
}


int OrcaUsbDriver::getPixelType(int *value) {
/**********************************************************************
**********************************************************************/
    int ret = 0;
    DCAMERR err;
    double data;

    if(!value)
        return -1;

    err = dcamprop_getvalue( hdcam, DCAM_IDPROP_IMAGE_PIXELTYPE, &data );
    if( !failed(err) )
    {
        *value = data;
    }
    else
    {
        printCameraError(cameraIndex, hdcam, err, "dcamprop_getvalue(DCAM_IDPROP_IMAGE_PIXELTYPE\n");
        ret = -1;
    }

    return ret;
}


int OrcaUsbDriver::setExposure(double value) {
/**********************************************************************
**********************************************************************/
    int ret = 0;
    DCAMERR err;

    // set exposure value in seconds
    err = dcamprop_setvalue( hdcam, DCAM_IDPROP_EXPOSURETIME, value );
    if( failed(err) )
    {
        printCameraError(cameraIndex, hdcam, err, "dcamprop_setvalue(DCAM_IDPROP_EXPOSURETIME)\n");
        ret = -1;
    }
    else
    {
    }

    return ret;
}


int OrcaUsbDriver::setExposureControl(int value) {
/**********************************************************************
**********************************************************************/
    int ret = 0;
    DCAMERR err;
    int prop;

    switch(value) 
    {
        case 0: /* free run mode */
            prop = DCAMPROP_TRIGGERSOURCE__INTERNAL;
            break;
        case 1: /* external trigger, sync In 1 */
            prop = DCAMPROP_TRIGGERSOURCE__EXTERNAL;
            break;
        default:
            return -1;
    }

    // set property value
    err = dcamprop_setvalue( hdcam, DCAM_IDPROP_TRIGGERSOURCE, prop );
    if( failed(err) )
    {
        printCameraError(cameraIndex, hdcam, err, "dcamprop_setvalue(DCAM_IDPROP_TRIGGERSOURCE)\n");
        ret = -1;
    }
    else
    {
    }

    return ret;
}


int OrcaUsbDriver::setTriggerControl(int value) {
/**********************************************************************
**********************************************************************/
    int ret = 0;
    DCAMERR err;
    int prop;

    switch(value) 
    {
        case 0: /* Edge Trigger */
            prop = DCAMPROP_TRIGGERACTIVE__EDGE;
            break;
        case 1: /* Level Trigger */
            prop = DCAMPROP_TRIGGERACTIVE__LEVEL;
            break;
        default:
            return -1;
    }

    // set property value
    err = dcamprop_setvalue( hdcam, DCAM_IDPROP_TRIGGERACTIVE, prop );
    if( failed(err) )
    {
        printCameraError(cameraIndex, hdcam, err, "dcamprop_setvalue(DCAM_IDPROP_TRIGGERACTIVE)\n");
        ret = -1;
    }
    else
    {
    }

    return ret;
}


int OrcaUsbDriver::setTriggerPolarity(int value) {
/**********************************************************************
**********************************************************************/
    int ret = 0;
    DCAMERR err;
    int prop;

    switch(value) 
    {
        case 0: /* Negative Polarity */
            prop = DCAMPROP_TRIGGERPOLARITY__NEGATIVE;
            break;
        case 1: /* Positive Polarity */
            prop = DCAMPROP_TRIGGERPOLARITY__POSITIVE;
            break;
        default:
            return -1;
    }

    // set property value
    err = dcamprop_setvalue( hdcam, DCAM_IDPROP_TRIGGERPOLARITY, prop );
    if( failed(err) )
    {
        printCameraError(cameraIndex, hdcam, err, "dcamprop_setvalue(DCAM_IDPROP_TRIGGERPOLARITY)\n");
        ret = -1;
    }
    else
    {
    }

    return ret;
}


int OrcaUsbDriver::getSensorMode(double *value) {
/**********************************************************************
  * SENSOR MODE (R/W) property allows you to specify the sensor mode of the camera.
  * The following values are predefined:
  * DCAMPROP_SENSORMODE__AREA
  *   "AREA"   The camera will output area image.
  * DCAMPROP_SENSORMODE__LINE
  *   "LINE"   The camera will output line image merged by DCAM_IDPROP_SENSORMODE_LINEBUNDLEHEIGHT.
  * DCAMPROP_SENSORMODE__TDI
  *    "TDI"  The camera will output line image with TDI technology merged by DCAM_IDPROP_SENSORMODE_LINEBUNDLEHEIGHT.
  * DCAMPROP_SENSORMODE__TDI_EXTENDED
  *   "TDI EXTENDED"   The mode is almost same as “TDI” but some cameras have 2 different LINE INTERVAL ranges.
  *   This mode has longer interval than “TDI”.
  * DCAMPROP_SENSORMODE__PROGRESSIVE
  *   "PROGRESSIVE"   The camera will output line image from top to bottom, or bottom to top with line speed control.
**********************************************************************/
    int ret = 0;
    DCAMERR err;
    double data;

    if (!value)
        return -1;

    err = dcamprop_getvalue( hdcam, DCAM_IDPROP_SENSORMODE, &data );
    if (!failed(err)) {
        *value = data;
    } else {
        printCameraError(cameraIndex, hdcam, err, "dcamprop_getvalue(DCAM_IDPROP_SENSORMODE)\n");
        ret = -1;
    }

    return ret;
}


int OrcaUsbDriver::setSensorMode(int value) {
/**********************************************************************
  * READOUT TIME (R/O) property returns frame read out time in seconds.
**********************************************************************/
    int ret = 0;
    DCAMERR err;

    // set property value
    err = dcamprop_setvalue( hdcam, DCAM_IDPROP_SENSORMODE, value );
    if( failed(err) )
    {
        printCameraError(cameraIndex, hdcam, err, "dcamprop_setvalue(DCAM_IDPROP_SENSORMODE)\n");
        ret = -1;
    }

    return ret;
}


/**
  * SHUTTER MODE (R/W) property allows you to get/set the shutter mode of CMOS sensor.
  */
int OrcaUsbDriver::getShutterMode(double *value) {
/**********************************************************************
**********************************************************************/
    int ret = 0;
    DCAMERR err;
    double data;

    if(!value)
        return -1;

    err = dcamprop_getvalue( hdcam, DCAM_IDPROP_SHUTTER_MODE, &data );
    if( !failed(err) )
    {
        *value = data;
    }
    else
    {
        printCameraError(cameraIndex, hdcam, err, "dcamprop_getvalue(DCAM_IDPROP_SHUTTER_MODE)\n");
        ret = -1;
    }

    return ret;
}


int OrcaUsbDriver::setShutterMode(int value) {
/**********************************************************************
**********************************************************************/
    std::string functionName = "setShutterMode";
    int ret = 0;
    DCAMERR err;
    int prop;

    switch(value) 
    {
        case 1: /* Global */
            prop = DCAMPROP_SHUTTER_MODE__GLOBAL;
            break;
        case 2: /* Rolling */
            prop = DCAMPROP_SHUTTER_MODE__ROLLING;
            break;
        default:
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s::%s: port %s: Error: invalid shutter mode %d\n", 
                    driverName.c_str(), functionName.c_str(), port_name.c_str(), value);
            return -1;
    }

    // set property value
    err = dcamprop_setvalue( hdcam, DCAM_IDPROP_SHUTTER_MODE, prop );
    if( failed(err) )
    {
        printCameraError(cameraIndex, hdcam, err, "dcamprop_setvalue(DCAM_IDPROP_SHUTTER_MODE)\n");
        ret = -1;
    }

    return ret;
}


int OrcaUsbDriver::getReadoutTime(double *value) {
/**********************************************************************
  * READOUT TIME (R/O) property returns frame read out time in seconds.
**********************************************************************/
    int ret = 0;
    DCAMERR err;
    double data;

    if(!value)
        return -1;

    err = dcamprop_getvalue( hdcam, DCAM_IDPROP_TIMING_READOUTTIME, &data );
    if( !failed(err) )
    {
        *value = data;
    }
    else
    {
        printCameraError(cameraIndex, hdcam, err, "dcamprop_getvalue(DCAM_IDPROP_TIMING_READOUTTIME)\n");
        ret = -1;
    }

    return ret;
}


int OrcaUsbDriver::getTimingExposure(double *value) {
/**********************************************************************
  * TIMING EXPOSURE (R/O) property returns the timing of exposure. 
  * This property can have one of following values:
  * DCAMPROP_TIMING_EXPOSURE__AFTERREADOUT
  *    "AFTER READOUT" 	The exposure starts after reading previous the frame completely.
  * DCAMPROP_TIMING_EXPOSURE__OVERLAPREADOUT
  *    "OVERLAP READOUT"   The exposure starts during reading the previous frame.
  * DCAMPROP_TIMING_EXPOSURE__ROLLING
  *   "ROLLING"   The exposure starts at each pixel after reading it at the previous frame.
  * DCAMPROP_TIMING_EXPOSURE__ALWAYS
  *   "ALWAYS"   The sensor is exposed always, even in reading out period.
  * DCAMPROP_TIMING_EXPOSURE__TDI
  *   "TDI"   The sensor is running as TDI sensor.
**********************************************************************/
    int ret = 0;
    DCAMERR err;
    double data;

    if(!value)
        return -1;

    err = dcamprop_getvalue( hdcam, DCAM_IDPROP_TIMING_EXPOSURE, &data );
    if( !failed(err) )
    {
        *value = data;
    }
    else
    {
        printCameraError(cameraIndex, hdcam, err, "dcamprop_getvalue(DCAM_IDPROP_TIMING_EXPOSURE)\n");
        ret = -1;
    }

    return ret;
}


int OrcaUsbDriver::getGlobalExposureDelay(double *value) {
/**********************************************************************
  * GLOBAL EXPOSURE DELAY (R/O) property returns how long GLOBAL EXPOSURE is delayed from beginning of EXPOSURE itself.
  * If the sensor does not have GLOBAL SHUTTER capability, GLOBAL EXPOSURE timing, 
  * which means all pixels on the sensor is exposed, is delayed. 
  *	This property is EFFECTIVE when DCAM_IDPROP_TRIGGER_GLOBAL_EXPOSURE is DELAYED.
**********************************************************************/
    int ret = 0;
    DCAMERR err;
    double data;

    if(!value)
        return -1;

    err = dcamprop_getvalue( hdcam, DCAM_IDPROP_TIMING_GLOBALEXPOSUREDELAY, &data );
    if( !failed(err) )
    {
        *value = data;
    }
    else
    {
        printCameraError(cameraIndex, hdcam, err, "dcamprop_getvalue(DCAM_IDPROP_TIMING_GLOBALEXPOSUREDELAY)\n");
        ret = -1;
    }

    return ret;
}


int OrcaUsbDriver::getInvalidExposurePeriod(double *value) {
/**********************************************************************
  * INVALID EXPOSURE PERIOD (R/O) value shows how long takes starting exposure from input trigger.
  * Because of its structure, some sensors cannot start exposure immediately.
  * There are various reasons but this property just tells how long it is.
  * This value does not include jitter of input trigger.
**********************************************************************/
    int ret = 0;
    DCAMERR err;
    double data;

    if(!value)
        return -1;

    err = dcamprop_getvalue( hdcam, DCAM_IDPROP_TIMING_INVALIDEXPOSUREPERIOD, &data );
    if( !failed(err) )
    {
        *value = data;
    }
    else
    {
        printCameraError(cameraIndex, hdcam, err, "dcamprop_getvalue(DCAM_IDPROP_TIMING_INVALIDEXPOSUREPERIOD)\n");
        ret = -1;
    }

    return ret;
}


int OrcaUsbDriver::getTriggerDelay(double *value) {
/**********************************************************************
  * TRIGGER DELAY (R/W) property can set delay time for using this timing inside of camera.
**********************************************************************/
    int ret = 0;
    DCAMERR err;
    double data;

    if(!value)
        return -1;

    err = dcamprop_getvalue( hdcam, DCAM_IDPROP_TRIGGERDELAY, &data );
    if( !failed(err) )
    {
        *value = data;
    }
    else
    {
        printCameraError(cameraIndex, hdcam, err, "dcamprop_getvalue(DCAM_IDPROP_TRIGGERDELAY)\n");
        ret = -1;
    }

    return ret;
}


int OrcaUsbDriver::setTriggerDelay(double value) {
/**********************************************************************
**********************************************************************/
    int ret = 0;
    DCAMERR err;

    // set trigger delay value in seconds
    err = dcamprop_setvalue( hdcam, DCAM_IDPROP_TRIGGERDELAY, value );
    if( failed(err) )
    {
        printCameraError(cameraIndex, hdcam, err, "dcamprop_setvalue(DCAM_IDPROP_TRIGGERDELAY)\n");
        ret = -1;
    }

    return ret;
}


int OrcaUsbDriver::getTriggerGlobalExposure(double *value) {
/**********************************************************************
  * TRIGGER GLOBAL EXPOSURE (R/W) property allows you to choose GLOBAL EXPOSURE option in some trigger modes. 
  * The following values are predefined:
  * DCAMPROP_TRIGGER_GLOBALEXPOSURE__ALWAYS
  *   "NONE"          Property not used. 
  *   "ALWAYS"        The sensor exposes globally during all exposure period. 
  *                   In this case, sensor is CCD or CMOS with global shutter.
  * DCAMPROP_TRIGGER_GLOBALEXPOSURE__DELAYED
  *   "DELAYED"       Global exposure is delayed from beginning of sensor exposure.
  *                   In this case, sensor is CMOS with rolling shutter.
  * DCAMPROP_TRIGGER_GLOBALEXPOSURE__EMULATE
  *   "EMULATE"       Global exposure is emulated. In this case, sensor is CMOS with rolling shutter,
  *                   but adding two frames at trigger. So the light source can illuminate
  *                   with very short period, this mode can emulate global shutter with rolling shutter mode.
  * DCAMPROP_TRIGGER_GLOBALEXPOSURE__GLOBALRESET
  *   "GLOBAL RESET"   Global reset is used. In this case, sensor is CMOS with rolling shutter,
  *                    but reset timing is globally same. So the light source can illuminate very soon
  *                    after trigger without waiting sensor reset time.
**********************************************************************/
    int ret = 0;
    DCAMERR err;
    double data;

    if(!value)
        return -1;

    err = dcamprop_getvalue( hdcam, DCAM_IDPROP_TRIGGER_GLOBALEXPOSURE, &data );
    if( !failed(err) )
    {
        *value = data;
    }
    else
    {
        printCameraError(cameraIndex, hdcam, err, "dcamprop_getvalue(DCAM_IDPROP_TRIGGER_GLOBALEXPOSURE)\n");
        ret = -1;
    }

    return ret;
}


int OrcaUsbDriver::setTriggerGlobalExposure(int value) {
/**********************************************************************
**********************************************************************/
    std::string functionName = "setTriggerGlobalExposure";
    int ret = 0;
    DCAMERR err;
    int prop;

    switch(value) 
    {
        case 1: /* None */
            prop = DCAMPROP_TRIGGER_GLOBALEXPOSURE__NONE;
            break;
        case 2: /* Always */
            prop = DCAMPROP_TRIGGER_GLOBALEXPOSURE__ALWAYS;
            break;
        case 3: /* Delayed */
            prop = DCAMPROP_TRIGGER_GLOBALEXPOSURE__DELAYED;
            break;
        case 4: /* Emulate */
            prop = DCAMPROP_TRIGGER_GLOBALEXPOSURE__EMULATE;
            break;
        case 5: /* Global Reset */
            prop = DCAMPROP_TRIGGER_GLOBALEXPOSURE__GLOBALRESET;
            break;
        default:
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s::%s: port %s: Error: invalid exposure mode %d\n", 
                    driverName.c_str(), functionName.c_str(), port_name.c_str(), value);
            return -1;
    }

    // set property value
    err = dcamprop_setvalue( hdcam, DCAM_IDPROP_TRIGGER_GLOBALEXPOSURE, prop );
    if( failed(err) )
    {
        printCameraError(cameraIndex, hdcam, err, "dcamprop_setvalue(DCAM_IDPROP_TRIGGER_GLOBALEXPOSURE)\n");
        ret = -1;
    }
    else
    {
    }

    return ret;
}


int OrcaUsbDriver::setBinning(int binning) {
/**********************************************************************
**********************************************************************/
    DCAMERR err;
    double value;

    err = dcamprop_queryvalue(hdcam, DCAM_IDPROP_BINNING, &value);

    if(failed(err))
    {
        printCameraError(cameraIndex, hdcam, err, "dcamprop_queryvalue()\n");
        return -1;
    }

    err = dcamprop_setvalue(hdcam, DCAM_IDPROP_BINNING, binning);

    if(failed(err))
    {
        printCameraError(cameraIndex, hdcam, err, "dcamprop_value()\n");
        return -1;
    }

    return 0;
}


int OrcaUsbDriver::getROI(int *hpos, int *hsize, int *vpos, int *vsize) {
/**********************************************************************
**********************************************************************/
    const std::string functionName = "getROI";
    DCAMERR err;
    double data;

    err = dcamprop_getvalue(hdcam, DCAM_IDPROP_SUBARRAYHPOS, &data);
    if (!failed(err)) {
        *hpos = data;
    } else {
        printCameraError(cameraIndex, hdcam, err, "dcamprop_getvalue(DCAM_IDPROP_SUBARRAYHPOS)\n");
        *hpos = MIN_X;
        return -1;
    }

    err = dcamprop_getvalue(hdcam, DCAM_IDPROP_SUBARRAYVPOS, &data);
    if (!failed(err)) {
        *vpos = data;
    } else {
        printCameraError(cameraIndex, hdcam, err, "dcamprop_getvalue(DCAM_IDPROP_SUBARRAYVPOS)\n");
        *vpos = MIN_Y;
        return -1;
    }

    err = dcamprop_getvalue(hdcam, DCAM_IDPROP_SUBARRAYHSIZE, &data);
    if (!failed(err)) {
        *hsize = data;
    } else {
        printCameraError(cameraIndex, hdcam, err, "dcamprop_getvalue(DCAM_IDPROP_SUBARRAYHSIZE)\n");
        *hsize = SIZE_X;
        return -1;
    }

    err = dcamprop_getvalue(hdcam, DCAM_IDPROP_SUBARRAYVSIZE, &data);
    if (!failed(err)) {
        *vsize = data;
    } else {
        printCameraError(cameraIndex, hdcam, err, "dcamprop_getvalue(DCAM_IDPROP_SUBARRAYVSIZE)\n");
        *vsize = SIZE_Y;
        return -1;
    }

    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
            "%s::%s: port %s: hpos=%d, hsize=%d, vpos=%d, vsize=%d\n", 
            driverName.c_str(), functionName.c_str(), port_name.c_str(),
            *hpos, *hsize, *vpos, *vsize);

    return 0;
}


int OrcaUsbDriver::setROI(int hpos, int hsize, int vpos, int vsize) {
/**********************************************************************
**********************************************************************/
    const std::string functionName = "setROI";
    DCAMERR err;

    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
            "%s::%s: port %s: hpos=%d, hsize=%d, vpos=%d, vsize=%d\n", 
            driverName.c_str(), functionName.c_str(), port_name.c_str(),
            hpos, hsize, vpos, vsize);

    err = dcamprop_setvalue(hdcam, DCAM_IDPROP_SUBARRAYMODE, DCAMPROP_MODE__OFF);
    if (failed(err))
    {
        printCameraError(cameraIndex, hdcam, err, "setROI\n");
        return -1;
    }

    err = dcamprop_setvalue(hdcam, DCAM_IDPROP_SUBARRAYHPOS, hpos);
    if (failed(err)) {
        printCameraError(cameraIndex, hdcam, err, "setROI\n");
        return -1;
    }

    err = dcamprop_setvalue(hdcam, DCAM_IDPROP_SUBARRAYVPOS, vpos);
    if (failed(err)) {
        printCameraError(cameraIndex, hdcam, err, "setROI\n");
        return -1;
    }

    err = dcamprop_setvalue(hdcam, DCAM_IDPROP_SUBARRAYHSIZE, hsize);
    if (failed(err)) {
        printCameraError(cameraIndex, hdcam, err, "setROI\n");
        return -1;
    }

    err = dcamprop_setvalue(hdcam, DCAM_IDPROP_SUBARRAYVSIZE, vsize);
    if (failed(err)) {
        printCameraError(cameraIndex, hdcam, err, "setROI\n");
        return -1;
    }

    err = dcamprop_setvalue(hdcam, DCAM_IDPROP_SUBARRAYMODE, DCAMPROP_MODE__ON);
    if (failed(err)) {
        printCameraError(cameraIndex, hdcam, err, "setROI\n");
        return -1;
    }

    return 0;
}


int OrcaUsbDriver::getCamIndex() {
/**********************************************************************
**********************************************************************/
    return cameraIndex;    
}


asynStatus OrcaUsbDriver::configureROI() {
/**********************************************************************
    Get image parameters from base class and validate.
**********************************************************************/
    const std::string functionName = "configureROI";
    int status = asynSuccess;

    status |= getIntegerParam(ADMaxSizeX, &maxSizeX);
    status |= getIntegerParam(ADMaxSizeY, &maxSizeY);

    status |= getIntegerParam(ADBinX, &binX);
    if (binX < 1) {
        binX = 1;
        status |= setIntegerParam(ADBinX, binX);
    }

    status |= getIntegerParam(ADBinY, &binY);
    if (binY < 1) {
        binY = 1;
        status |= setIntegerParam(ADBinY, binY);
    }

    status |= getIntegerParam(ADMinX, &minX);
    if (minX <= 0) {
        minX = 1; 
        status |= setIntegerParam(ADMinX, minX);
    } else if (minX > maxSizeX-1) {
        minX = maxSizeX-1;
        status |= setIntegerParam(ADMinX, minX);
    }

    status |= getIntegerParam(ADMinY, &minY);
    if (minY <= 0) {
        minY = 1; 
        status |= setIntegerParam(ADMinY, minY);
    } else if (minY > maxSizeY-1) {
        minY = maxSizeY-1;
        status |= setIntegerParam(ADMinY, minY);
    }

    status |= getIntegerParam(ADSizeX, &sizeX);
    if (minX+sizeX > maxSizeX) {
        sizeX = maxSizeX - minX;
        status |= setIntegerParam(ADSizeX, sizeX);
        asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
                "%s::%s: port %s: sizeX changed to %d\n", 
                driverName.c_str(), functionName.c_str(), port_name.c_str(), sizeX);
    }
    if (sizeX <= 0) {
        sizeX = 1;
        status |= setIntegerParam(ADSizeX, sizeX);
    }

    status |= getIntegerParam(ADSizeY, &sizeY);
    if (minY+sizeY > maxSizeY) {
        sizeY = maxSizeY - minY;
        status |= setIntegerParam(ADSizeY, sizeY);
        asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
                "%s::%s: port %s: sizeY changed to %d\n", 
                driverName.c_str(), functionName.c_str(), port_name.c_str(), sizeY);
    }
    if (sizeY <= 0) {
        sizeY = 1;
        status |= setIntegerParam(ADSizeY, sizeY);
    }
        
    status |= setIntegerParam(NDArraySizeX, sizeX);
    status |= setIntegerParam(NDArraySizeY, sizeY);
    status |= setIntegerParam(NDArraySize, sizeX*sizeY*sizeof(epicsUInt16));

    if (asynStatus(status) != asynSuccess) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s::%s: port %s: Error setting parameter\n", 
                driverName.c_str(), functionName.c_str(), port_name.c_str());
    }

    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
            "%s::%s: port %s: minX=%d, minY=%d, sizeX=%d, sizeY=%d, binX=%d, binY=%d\n", 
            driverName.c_str(), functionName.c_str(), port_name.c_str(),
            minX, minY, sizeX, sizeY, binX, binY);

    callParamCallbacks();
    return asynStatus(status);
}


void OrcaUsbDriver::printCameraError(int camIndex, HDCAM hdcam, DCAMERR errid, const char* apiname) {
/**********************************************************************
**********************************************************************/
    std::string functionName = "printCameraError";
        char errtext[ 256 ];
        DCAMERR err;
        DCAMDEV_STRING  param;

        memset(&param, 0, sizeof(param));
        param.size      = sizeof(param);
        param.text      = errtext;
        param.textbytes = sizeof(errtext);
        param.iString   = errid;

        err = dcamdev_getstring(hdcam, &param);

        if (!failed(err)) {
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s::%s: port %s: FAILED(#%d): 0x%08X: %s @ %s\n", 
                    driverName.c_str(), functionName.c_str(), port_name.c_str(),
                    camIndex, errid, errtext, apiname);
        } else {
            asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
                    "%s::%s: port %s: Error on cam %d getting error string for %s error\n", 
                    driverName.c_str(), functionName.c_str(), port_name.c_str(),
                    camIndex, apiname);
        }
}


int OrcaUsbDriver::findCamera(const char* cameraId) {
/**********************************************************************
**********************************************************************/
    std::string functionName = "findCamera";
    DCAMERR err;
    bool cameraFound = false;
    char serial[64];

    for (int i=0; i<deviceCount; i++) {
        // open and check unused cameras only
        if (openCameras[i] == 0) {
            // open device
            DCAMDEV_OPEN devopen;
            memset( &devopen, 0, sizeof(devopen) );
            devopen.size = sizeof(devopen);
            devopen.index = i;

            asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
                    "%s::%s: %s: Trying to open camera... %d\n",
                    driverName.c_str(), functionName.c_str(), port_name.c_str(), i);
            err = dcamdev_open(&devopen);
            if (failed(err)) {
                asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
                        "%s::%s: %s: Error opening camera %d\n",
                        driverName.c_str(), functionName.c_str(), port_name.c_str(), i);
                continue;
            } 

            asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
                    "%s::%s: %s: Opened camera %d for ID check...\n",
                    driverName.c_str(), functionName.c_str(), port_name.c_str(), i);

            hdcam = devopen.hdcam;

            if (getCameraSerial(serial) != -1) {
                if ((strcmp(cameraId, "0") == 0) && (strcmp(serial, "0") != 0)) {
                    // Don't match camera by serial number; use next available camera
                    cameraFound = true;
                    openCameras[i] = 1;
                    epicsPrintf("Camera %d with %s found\n", i, serial);
                    break;
                } else {
                    // Match camera by serial number
                    if (strstr(serial, cameraId)) {
                        // camera with specified serial number found
                        cameraFound = true;
                        openCameras[i] = 1;
                        epicsPrintf("Camera %d with ID %s found\n", i, cameraId);
                        break;
                    } else {
                        // close camera with unmatching serial number 
                        dcamdev_close(hdcam);
                        hdcam = NULL;
                        asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
                                "%s::%s: %s: Unmatching camera %d closed\n",
                                driverName.c_str(), functionName.c_str(), port_name.c_str(), i);
                    }
                }
            } else {
                // close camera if serial number could not be retreived
                asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
                        "%s::%s: %s: Could not get id for camera %d\n",
                        driverName.c_str(), functionName.c_str(), port_name.c_str(), i);
                dcamdev_close(hdcam);
                hdcam = NULL;
            }
        } else {
            asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
                    "%s::%s: %s: Camera %d already in use\n",
                    driverName.c_str(), functionName.c_str(), port_name.c_str(), i);
        }

    }  // End for loop

    if (!cameraFound) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s::%s: port %s: Error: no unopen camera with ID %s found\n", 
                driverName.c_str(), functionName.c_str(), port_name.c_str(), cameraId);
        if (hdcam != NULL) {
            dcamdev_close(hdcam);
            hdcam = NULL;
        }
        return 0;
    }

    return 1;
}


void OrcaUsbDriver::dataTask(void) {
/**********************************************************************
    Main data task; runs in a separate thread.
**********************************************************************/
    std::string functionName = "dataTask";
    DCAMERR err;
    size_t dims[2] = {SIZE_X, SIZE_Y};
    NDArray *pArray = NULL;
    int rowbytes;
    int imageCounter;

    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
            "%s::%s: %s: Started dataTask...\n",
            driverName.c_str(), functionName.c_str(), port_name.c_str());

    this->lock();
    while (!exit_loop) {
        // Wait for an event signal (e.g. acquisition)
        asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
                "%s::%s: %s: Waiting for dataEvent...\n",
                driverName.c_str(), functionName.c_str(), port_name.c_str());
        this->unlock();
        epicsEventWait(dataEvent);
        this->lock();
        asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
                "%s::%s: %s: dataEvent triggered...\n",
                driverName.c_str(), functionName.c_str(), port_name.c_str());
        if (exit_loop) break;

        // Prepare image mode
        int imageMode, numImages;
        getIntegerParam(ADImageMode, &imageMode);
        getIntegerParam(ADNumImages, &numImages);
        if (imageMode == ADImageSingle) {
            this->framesRemaining = 1;
        } else if (imageMode == ADImageMultiple) {
            this->framesRemaining = numImages;
        } else {
            this->framesRemaining = -1;
        }

        // Get/set image parameters
        int minX_tmp, sizeX_tmp, minY_tmp, sizeY_tmp;
        getIntegerParam(ADMinX, &minX_tmp);
        getIntegerParam(ADSizeX, &sizeX_tmp);
        getIntegerParam(ADMinY, &minY_tmp);
        getIntegerParam(ADSizeY, &sizeY_tmp);
        if (setROI(minX_tmp, sizeX_tmp, minY_tmp, sizeY_tmp) == -1) {
            // Failed to set ROI
            acquire = false;
            setIntegerParam(ADAcquire, acquire);
            setIntegerParam(ADStatus, ADStatusError);
            setStringParam(ADStatusMessage, "Stopped acquisition");
            callParamCallbacks();
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s::%s: %s: Stopped acquisition: setROI failed\n",
                    driverName.c_str(), functionName.c_str(), port_name.c_str());
            continue;
        }
        if (getROI(&minX_tmp, &sizeX_tmp, &minY_tmp, &sizeY_tmp) == 0) {
            // Got parameters from camera, update driver with actual values
            setIntegerParam(ADMinX, minX_tmp);
            setIntegerParam(ADSizeX, sizeX_tmp);
            setIntegerParam(ADMinY, minY_tmp);
            setIntegerParam(ADSizeY, sizeY_tmp);
            setIntegerParam(NDArraySizeX, sizeX_tmp);
            setIntegerParam(NDArraySizeY, sizeY_tmp);
            setIntegerParam(NDArraySize, sizeX_tmp*sizeY_tmp*sizeof(epicsUInt16));
        }

        // Reset counters
        setIntegerParam(ADNumImagesCounter, 0);
        setIntegerParam(ADNumExposuresCounter, 0);
        callParamCallbacks();

        // Configure camera event wait
        DCAMWAIT_OPEN waitopen;
        memset(&waitopen, 0, sizeof(waitopen));
        waitopen.size  = sizeof(waitopen);
        waitopen.hdcam = hdcam;
        err = dcamwait_open(&waitopen);
        if (failed(err)) {
            printCameraError(cameraIndex, hdcam, err, "dcamwait_open()\n");
            return;
        }
        HDCAMWAIT hwait = waitopen.hwait;

        // Allocate camera buffers
        err = dcambuf_alloc(hdcam, 10);
        if (failed(err)) {
            printCameraError(cameraIndex, hdcam, err, "dcambuf_alloc()\n");
            dcamwait_close(hwait);
            return;
        }

        // Configure camera capturing
        err = dcamcap_start(hdcam, DCAMCAP_START_SEQUENCE);
        if (failed(err)) {
            printCameraError(cameraIndex, hdcam, err, "dcamcap_start()\n");
            dcambuf_release(hdcam);
            dcamwait_close(hwait);
            return;
        }

        // Enable camera event wait
        DCAMWAIT_START waitstart;
        memset(&waitstart, 0, sizeof(waitstart));
        waitstart.size      = sizeof(waitstart);
        waitstart.eventmask = DCAMWAIT_CAPEVENT_FRAMEREADY;
        waitstart.timeout   = 5000;

        while(acquire) {
            asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
                    "%s::%s: %s: Acquire loop...\n",
                    driverName.c_str(), functionName.c_str(), port_name.c_str());
            this->unlock();
            err = dcamwait_start(hwait, &waitstart);
            this->lock();
            if (failed(err)) {
                //printCameraError(cameraIndex, hdcam, err, "dcamwait_start()\n");
                //acquire = 0;
                //setIntegerParam(ADAcquire, acquire);
                setIntegerParam(ADStatus, ADStatusError);
                setStringParam(ADStatusMessage, "Error: dcamwait_start");
                callParamCallbacks();
                //break;
                dcamcap_stop(hdcam);
                continue;
            }
            
            DCAMCAP_TRANSFERINFO captransferinfo;
            memset(&captransferinfo, 0, sizeof(captransferinfo));
            captransferinfo.size = sizeof(captransferinfo);

            err = dcamcap_transferinfo(hdcam, &captransferinfo);
            if (failed(err)) {
                printCameraError(cameraIndex, hdcam, err, "dcamcap_transferinfo()\n");
                continue;
            }

            if (captransferinfo.nFrameCount < 1) {
                asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                        "%s::%s: %s: Error: no image captured\n",
                        driverName.c_str(), functionName.c_str(), port_name.c_str());
                continue;
            }

            if (!acquire) {
                continue;
            }

            // Access image
            DCAMBUF_FRAME bufframe;
            memset(&bufframe, 0, sizeof(bufframe));
            bufframe.size = sizeof(bufframe);
            // iFrame: set to index of image buffer, can be set to -1 to retrieve latest captured image
            bufframe.iFrame = -1;
            getRowBytes(&rowbytes);
            int esizex, esizey;
            getEffectiveSizeX(&esizex);
            getEffectiveSizeY(&esizey);
            asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
                    "%s::%s: %s: sizeX_tmp=%d, sizeY_tmp=%d, esizex=%d, esizey=%d, rowbytes=%d\n",
                    driverName.c_str(), functionName.c_str(), port_name.c_str(),
                    sizeX_tmp, sizeY_tmp, esizex, esizey, rowbytes);
    
            int ndims = 2;
            dims[0] = sizeX_tmp;
            dims[1] = sizeY_tmp;
            pArray = this->pNDArrayPool->alloc(ndims, dims, NDUInt16, 0, NULL);

#if USE_COPYFRAME
            bufframe.buf	  = (void*)pArray->pData;
            bufframe.rowbytes = rowbytes;
            bufframe.left	  = 0;
            bufframe.top	  = 0;
            bufframe.width	  = sizeX_tmp;
            bufframe.height	  = sizeY_tmp;

            err = dcambuf_copyframe(hdcam, &bufframe);
            if (failed(err)) {
                printCameraError(cameraIndex, hdcam, err, "dcambuf_copyframe()\n");
            }
#else
		    err = dcambuf_lockframe(hdcam, &bufframe);
		    if (failed(err)) {
                printCameraError(cameraIndex, hdcam, err, "dcambuf_lockframe()\n");
		    }
            // TODO: add ROI code
#endif
            
            updateTimeStamp(&pArray->epicsTS);
            pArray->uniqueId = PULSEID(pArray->epicsTS);
            getAttributes(pArray->pAttributeList);

            {   /* provide POSIX timestamp for the image native timestamp */
                timespec ts;
                epicsTimeToTimespec(&ts, &pArray->epicsTS);
                pArray->timeStamp = (double)ts.tv_sec + ((double)ts.tv_nsec * 1.0e-09);
            }

            doCallbacksGenericPointer(pArray, NDArrayData, 0);
            pArray->release();

            if (this->framesRemaining > 0) {  // Keep looping
                this->framesRemaining--;
            }
            
            if (this->framesRemaining == 0)  {  // Stop looping
                acquire = false;
                setIntegerParam(ADAcquire, acquire);
            }

            getIntegerParam(ADNumImagesCounter, &imageCounter);
            imageCounter++;
            setIntegerParam(ADNumImagesCounter, imageCounter);

            getIntegerParam(NDArrayCounter, &imageCounter);
            imageCounter++;
            setIntegerParam(NDArrayCounter, imageCounter);

            callParamCallbacks();
            asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
                    "%s::%s: %s: Exiting acquire loop...\n",
                    driverName.c_str(), functionName.c_str(), port_name.c_str());
        }  // End of acquire loop

        dcamcap_stop(hdcam);
        dcambuf_release(hdcam);
        dcamwait_close(hwait);
        
        setIntegerParam(ADStatus, ADStatusIdle);
        callParamCallbacks();
    } // End of infinite loop

    exited = true;
    asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
            "%s::%s: %s: Exiting dataTask...\n",
            driverName.c_str(), functionName.c_str(), port_name.c_str());
    this->unlock();
}  // End dataTask


//-------------------------------------------------------------------------
// ADDriver function overrides
//-------------------------------------------------------------------------
asynStatus OrcaUsbDriver::writeInt32(asynUser *pasynUser, epicsInt32 value) {
/**********************************************************************
    * Called when asyn clients call pasynInt32->write().
    * This function performs actions for some parameters, including ADAcquire, ADBinX, etc.
    * For all parameters it sets the value in the parameter library and calls any registered callbacks.
    * \param[in] pasynUser pasynUser structure that encodes the reason and address.
    * \param[in] value Value to write.
**********************************************************************/
    static std::string functionName = "writeInt32";
    int function = pasynUser->reason;
    int status = asynSuccess;

    /* Set the parameter and readback in the parameter library.  This may be overwritten when we read back the
     * status at the end, but that's OK */
    status = setIntegerParam(function, value);

    if (function == ADAcquire) {
        // acquire on connected cameras only
        if (cameraAvailable != 0) {
            if (value && !acquire) {
                acquire = true;
                status |= setIntegerParam(ADAcquire, acquire);
                status |= setIntegerParam(ADStatus, ADStatusAcquire);
                setStringParam(ADStatusMessage, "Acquiring");
                epicsEventSignal(dataEvent);
            }

            if (!value && acquire) {
                acquire = false;
                status |= setIntegerParam(ADAcquire, acquire);
            }
        } else {
            asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
                    "%s:%s: %s: Camera #%d not available\n", 
                    driverName.c_str(), functionName.c_str(), port_name.c_str(), cameraIndex);
        }
    }  else if ((function == ADSizeX) ||
                (function == ADSizeY) ||
                (function == ADMinX) ||
                (function == ADMinY) ||
                (function == ADBinX) ||
                (function == ADBinY)) {
        if (!acquire) {
            configureROI();
        }
    } else if (function == ADTriggerMode) {
        if (setExposureControl(value) != -1)
            status |= setIntegerParam(ADTriggerMode, value);
    } else if (function == cameraTrg) {
        if (setTriggerControl(value) != -1)
            status |= setIntegerParam(cameraTrg, value);
    } else if (function == cameraTrgPolarity) {
        if (setTriggerPolarity(value) != -1)
            status |= setIntegerParam(cameraTrgPolarity, value);
    } else if (function == cameraTrgGlobalExposure) {
        if (setTriggerGlobalExposure(value) != -1)
            status |= setIntegerParam(cameraTrgGlobalExposure, value); 
    } else if (function == cameraReadStat) {
        // update camera connection status
        int stat = getCameraStatus();
        status |= setIntegerParam(cameraStatus, stat);
    } else {
        /* If this is not a parameter we have handled call the base class */
        if (function < FIRST_ORCA_PARAM) {
            status |= ADDriver::writeInt32(pasynUser, value);
        }
    }

    if (status) {
        asynPrint(pasynUser, ASYN_TRACE_ERROR, 
                "%s:%s: %s: Error: status=%d, function=%d, value=%d\n", 
                driverName.c_str(), functionName.c_str(), port_name.c_str(), status, function, value);
    } else {
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, 
              "%s:%s: %s: function=%d, value=%d\n", 
                driverName.c_str(), functionName.c_str(), port_name.c_str(), function, value);
    }

    callParamCallbacks();
    return (asynStatus)status;
}


asynStatus OrcaUsbDriver::writeFloat64(asynUser *pasynUser, epicsFloat64 value) {
/**********************************************************************
    * Called when asyn clients call pasynFloat64->write().
    * This function performs actions for some parameters.
    * For all parameters it sets the value in the parameter library and calls any registered callbacks.
    * \param[in] pasynUser pasynUser structure that encodes the reason and address.
    * \param[in] value Value to write.
**********************************************************************/
    static std::string functionName = "writeFloat64";
    int function = pasynUser->reason;
    int status = asynSuccess;

    /* Set the parameter and readback in the parameter library.  This may be overwritten when we read back the
     * status at the end, but that's OK */
    status = setDoubleParam(function, value);

    if (function == ADAcquireTime) {
        double tmpfloat64;
        setExposure(value);
        if (getActualExposure(&tmpfloat64) != -1) 
            status |= setDoubleParam(ADAcquireTime, tmpfloat64);
    } else {
        /* If this is not a parameter we have handled call the base class */
        if (function < FIRST_ORCA_PARAM) {
            status |= ADDriver::writeFloat64(pasynUser, value);
        }
    }

    if (status) {
        asynPrint(pasynUser, ASYN_TRACE_ERROR, 
                "%s:%s: %s: Error: status=%d, function=%d, value=%f\n", 
                driverName.c_str(), functionName.c_str(), port_name.c_str(), status, function, value);
    } else {
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER, 
              "%s:%s: %s: function=%d, value=%f\n", 
                driverName.c_str(), functionName.c_str(), port_name.c_str(), function, value);
    }

    callParamCallbacks();
    return (asynStatus)status;
}

//-------------------------------------------------------------------------
// End ADDriver function overrides
//-------------------------------------------------------------------------


void OrcaUsbDriver::report(FILE *fp, int details) {
/**********************************************************************
  * Report status of the driver.
  * Prints details about the detector in use if details > 0.
  * It then calls the ADDriver::report() method.
  * \param[in] fp File pointed passed by caller where the output is written to.
  * \param[in] details Controls the level of detail in the report.
**********************************************************************/
    std::string functionName = "report";
    char str[128];
    int xsize, ysize;

    fprintf(fp, "Orca  cam %d, port = %s\n", cameraIndex, port_name.c_str());
    if (details > 0) {
        if (getCameraName(str) != -1)
            fprintf(fp, "  Model: %s\n", str);
        if (getCameraInfo(str) != -1)
            fprintf(fp, "  Series: %s\n", str);
        if (getCameraSerial(str) != -1) 
            fprintf(fp, "  Serial number: %s\n", str); 
        if (getCameraFirmware(str) != -1) 
            fprintf(fp, "  Firmware version: %s\n", str);

        getIntegerParam(ADMaxSizeX, &xsize);
        getIntegerParam(ADMaxSizeY, &ysize);
        fprintf(fp, "  X pixels: %d\n", xsize);
        fprintf(fp, "  Y pixels: %d\n", ysize);
    }
    // Call the base class method
    ADDriver::report(fp, details);
}


/**********************************************************************
 * External configuration function for OrcaUsb.
 * Envokes the constructor to create a new OrcaUsb object
 * This is the function that initializes the driver, and is called in the IOC startup script
 *
 * \params[in]: all passed into constructor
 * \return: status
**********************************************************************/
extern "C" {
int OrcaConfig(char *portName, char *cameraId, int maxBuffers, size_t maxMemory, int priority, int stackSize)
{
    new OrcaUsbDriver(portName, cameraId, maxBuffers, maxMemory, priority, stackSize);
    return(asynSuccess);
}

/**********************************************************************
    Code for iocsh registration
**********************************************************************/
static const iocshArg OrcaConfigArg0 = {"Port name",   iocshArgString};
static const iocshArg OrcaConfigArg1 = {"CameraId",    iocshArgString};
static const iocshArg OrcaConfigArg2 = {"maxBuffers",  iocshArgInt};
static const iocshArg OrcaConfigArg3 = {"maxMemory",   iocshArgInt};
static const iocshArg OrcaConfigArg4 = {"priority",    iocshArgInt};
static const iocshArg OrcaConfigArg5 = {"stackSize",   iocshArgInt};
static const iocshArg * const OrcaConfigArgs[] = { &OrcaConfigArg0,
                                                   &OrcaConfigArg1,
                                                   &OrcaConfigArg2,
                                                   &OrcaConfigArg3,
                                                   &OrcaConfigArg4,
                                                   &OrcaConfigArg5 };

static const iocshFuncDef OrcaConfigFuncDef = {"OrcaConfig", 6, OrcaConfigArgs};

static void  OrcaConfigCallFunc(const iocshArgBuf *args) {
    OrcaConfig(args[0].sval, args[1].sval, args[2].ival, 
               args[3].ival, args[4].ival, args[5].ival);
}

static void OrcaConfigRegister(void) {
    iocshRegister(&OrcaConfigFuncDef, OrcaConfigCallFunc);
}


epicsExportRegistrar(OrcaConfigRegister);
}  // End extern "C"

