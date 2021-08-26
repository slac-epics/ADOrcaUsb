/*
 * Area Detector Driver for the Orca USB
 *
 * Author Janos, Vamosi
 * Date   Feb-28-2021
 *
 */

#include <stdio.h>
#include <string.h>
#include <string>
#include <unistd.h>

#include <ellLib.h>
#include <epicsTime.h>
#include <epicsThread.h>
#include <epicsEvent.h>
#include <epicsString.h>
#include <iocsh.h>
#include <epicsExit.h>
#include <epicsMutex.h>
#include <epicsEvent.h>
#include <epicsExit.h>

#include <epicsExport.h>

extern "C" {
#include "perfMeasure.h"
//#include "edtinc.h"
//int deIntlv_MidTop_Line16(u_short *src, int width, int rows, u_short *dest);
//int pdv_query_serial(PdvDev * pdv_p, char *cmd, char **resp);
//int ErGetTicks(int, unsigned*);
#define LOWER_17_BIT_MASK       (0x0001FFFF)    /* (2^17)-1            */
#define PULSEID(time)           ((time).nsec & LOWER_17_BIT_MASK)
}

#include "orcaUsbDriver.h"

//static const char *driverName = "OrcaUsb";

static void dataTaskC(void *drvPvt)
{
    OrcaUsbDriver *pOrca = (OrcaUsbDriver*) drvPvt;
    pOrca->dataTask();
}

/**
 * Callback function called when IOC is terminated.
 * Deletes created object and frees Orca context.
 *
 * \params[in]: pointer to the OrcaUsb object created in OrcaUsbConfig
 */
static void exitHookC(void *drvPvt)
{
    OrcaUsbDriver *pOrca = (OrcaUsbDriver*) drvPvt;

    printf("#%d: Calling exit hook for OrcaUsb driver\n", pOrca->getCamIndex());
    pOrca->exitHook();
}

int OrcaUsbDriver::getCameraName(char *value)
{
    int  ret = 0;
    DCAMERR err;
    char data[256];

    if(!value)
        return -1;

    DCAMDEV_STRING param;
    memset( &param, 0, sizeof(param) );
    param.size = sizeof(param);
    param.text = data;
    param.textbytes = sizeof(data);
    param.iString = DCAM_IDSTR_MODEL;

    err = dcamdev_getstring( hdcam, &param );
    if( !failed(err) )
    {
        strcpy(value, data);
    }
    else
    {
        //printf( "Error: get DCAM_IDSTR_MODEL\n" );
        printCameraError(cameraIndex, hdcam, err, "dcamprop_getvalue(DCAM_IDSTR_MODEL)\n");
        ret = -1;
    }

    return ret;
}

int OrcaUsbDriver::getCameraSerial(char *value)
{
    int  ret = 0;
    DCAMERR err;
    char data[256];

    if(!value)
        return -1;

    DCAMDEV_STRING param;
    memset( &param, 0, sizeof(param) );
    param.size = sizeof(param);
    param.text = data;
    param.textbytes = sizeof(data);
    param.iString = DCAM_IDSTR_CAMERAID;

    err = dcamdev_getstring( hdcam, &param );
    if( !failed(err) )
    {
        strcpy(value, data);
    }
    else
    {
        //printf( "Error: get DCAM_IDSTR_CAMERAID\n" );
        printCameraError(cameraIndex, hdcam, err, "dcamprop_getvalue(DCAM_IDSTR_CAMERAID)\n");
        ret = -1;
    }

    return ret;
}

int OrcaUsbDriver:: getCameraFirmware(char *value)
{
    int  ret = 0;
    DCAMERR err;
    char data[256];

    if(!value)
        return -1;

    DCAMDEV_STRING param;
    memset( &param, 0, sizeof(param) );
    param.size = sizeof(param);
    param.text = data;
    param.textbytes = sizeof(data);
    param.iString = DCAM_IDSTR_CAMERAVERSION;

    err = dcamdev_getstring( hdcam, &param );
    if( !failed(err) )
    {
        strcpy(value, data);
    }
    else
    {
        //printf( "Error: get DCAM_IDSTR_CAMERAVERSION\n" );
        printCameraError(cameraIndex, hdcam, err, "dcamprop_getvalue(DCAM_IDSTR_CAMERAVERSION)\n");
        ret = -1;
    }

    return ret;
}

int OrcaUsbDriver::getCameraInfo(char *value)
{
    int  ret = 0;
    DCAMERR err;
    char data[256];

    if(!value)
        return -1;

    DCAMDEV_STRING param;
    memset( &param, 0, sizeof(param) );
    param.size = sizeof(param);
    param.text = data;
    param.textbytes = sizeof(data);
    param.iString = DCAM_IDSTR_CAMERA_SERIESNAME;

    err = dcamdev_getstring( hdcam, &param );
    if( !failed(err) )
    {
        strcpy(value, data);
    }
    else
    {
        //printf( "Error: get DCAM_IDSTR_CAMERA_SERIESNAME\n" );
        printCameraError(cameraIndex, hdcam, err, "dcamprop_getvalue(DCAM_IDSTR_SERIESNAME)\n");
        ret = -1;
    }

    return ret;
}

int OrcaUsbDriver::getActualExposure(double *value)
{
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
        //printf( "Error: get DCAM_IDPROP_EXPOSURETIME\n" );
        printCameraError(cameraIndex, hdcam, err, "dcamprop_getvalue(DCAM_IDPROP_EXPOSURETIME)\n");
        ret = -1;
    }

    return ret;
}

int OrcaUsbDriver::getEffectiveSizeX(int *value)
{
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
        //printf( "Error: get DCAM_IDPROP_IMAGE_WIDTH\n" );
        printCameraError(cameraIndex, hdcam, err, "dcamprop_getvalue(DCAM_IDPROP_IMAGE_WIDTH)\n");
        ret = -1;
    }

    return ret;
}

int OrcaUsbDriver::getRowBytes(int *value)
{
    int ret = 0;
    DCAMERR err;
    double data;

    if(!value)
        return -1;

    err = dcamprop_getvalue( hdcam, DCAM_IDPROP_IMAGE_ROWBYTES, &data );
    if( !failed(err) )
    {
        *value = data;
    }
    else
    {
        //printf( "Error: get DCAM_IDPROP_IMAGE_ROWBYTES\n" );
        printCameraError(cameraIndex, hdcam, err, "dcamprop_getvalue(DCAM_IDPROP_IMAGE_ROWBYTES)\n");
        ret = -1;
    }

    return ret;
}

int OrcaUsbDriver::getEffectiveSizeY(int *value)
{
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
        //printf( "Error: get DCAM_IDPROP_IMAGE_HEIGHT\n" );
        printCameraError(cameraIndex, hdcam, err, "dcamprop_getvalue(DCAM_IDPROP_IMAGE_HEIGHT)\n");
        ret = -1;
    }

    return ret;
}

int OrcaUsbDriver::getPixelType(int *value)
{
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
        //printf( "Error: get DCAM_IDPROP_IMAGE_PIXELTYPE\n" );
        printCameraError(cameraIndex, hdcam, err, "dcamprop_getvalue(DCAM_IDPROP_IMAGE_PIXELTYPE\n");
        ret = -1;
    }

    return ret;
}

int OrcaUsbDriver::setExposure(double value)
{
    int ret = 0;
    DCAMERR err;

    // set exposure value in seconds
    err = dcamprop_setvalue( hdcam, DCAM_IDPROP_EXPOSURETIME, value );
    if( failed(err) )
    {
        //printf( "Error: set DCAM_IDPROP_EXPOSURETIME\n" );
        printCameraError(cameraIndex, hdcam, err, "dcamprop_setvalue(DCAM_IDPROP_EXPOSURETIME)\n");
        ret = -1;
    }
    else
    {
        //printf( "#%d: DCAM_IDPROP_EXPOSURETIME set to %f\n", cameraIndex, value);
    }

    return ret;
}

int OrcaUsbDriver::setExposureControl(int value)
{
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
        //printf( "Error: set DCAM_IDPROP_TRIGGERSOURCE\n" );
        printCameraError(cameraIndex, hdcam, err, "dcamprop_setvalue(DCAM_IDPROP_TRIGGERSOURCE)\n");
        ret = -1;
    }
    else
    {
        //printf( "#%d: DCAM_IDPROP_TRIGGERSOURCE set to %d\n", cameraIndex, prop);
    }

    return ret;
}

int OrcaUsbDriver::setTriggerControl(int value)
{
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
        //printf( "Error: set DCAM_IDPROP_TRIGGERACTIVE\n" );
        printCameraError(cameraIndex, hdcam, err, "dcamprop_setvalue(DCAM_IDPROP_TRIGGERACTIVE)\n");
        ret = -1;
    }
    else
    {
        //printf( "#%d: DCAM_IDPROP_TRIGGERACTIVE set to %d\n", cameraIndex, prop);
    }

    return ret;
}

int OrcaUsbDriver::setTriggerPolarity(int value)
{
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
        //printf( "Error: set DCAM_IDPROP_TRIGGERPOLARITY\n" );
        printCameraError(cameraIndex, hdcam, err, "dcamprop_setvalue(DCAM_IDPROP_TRIGGERPOLARITY)\n");
        ret = -1;
    }
    else
    {
        //printf( "#%d: DCAM_IDPROP_TRIGGERPOLARITY set to %d\n", cameraIndex, prop);
    }

    return ret;
}

/**
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
  */
int OrcaUsbDriver::getSensorMode(double *value)
{
    int ret = 0;
    DCAMERR err;
    double data;

    if(!value)
        return -1;

    err = dcamprop_getvalue( hdcam, DCAM_IDPROP_SENSORMODE, &data );
    if( !failed(err) )
    {
        *value = data;
    }
    else
    {
        //printf( "Error: get DCAM_IDPROP_SENSORMODE\n" );
        printCameraError(cameraIndex, hdcam, err, "dcamprop_getvalue(DCAM_IDPROP_SENSORMODE)\n");
        ret = -1;
    }

    return ret;
}

/**
  * READOUT TIME (R/O) property returns frame read out time in seconds.
  */
int OrcaUsbDriver::setSensorMode(int value)
{
    int ret = 0;
    DCAMERR err;

    // set property value
    err = dcamprop_setvalue( hdcam, DCAM_IDPROP_SENSORMODE, value );
    if( failed(err) )
    {
        //printf( "Error: set DCAM_IDPROP_SENSORMODE\n" );
        printCameraError(cameraIndex, hdcam, err, "dcamprop_setvalue(DCAM_IDPROP_SENSORMODE)\n");
        ret = -1;
    }

    return ret;
}

/**
  * SHUTTER MODE (R/W) property allows you to get/set the shutter mode of CMOS sensor.
  */
int OrcaUsbDriver::getShutterMode(double *value)
{
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
        //printf( "Error: get DCAM_IDPROP_SHUTTER_MODE\n" );
        printCameraError(cameraIndex, hdcam, err, "dcamprop_getvalue(DCAM_IDPROP_SHUTTER_MODE)\n");
        ret = -1;
    }

    return ret;
}

int OrcaUsbDriver::setShutterMode(int value)
{
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
            printf( "#%d: Error: invalid value for DCAM_IDPROP_SHUTTER_MODE\n", cameraIndex );
            return -1;
    }

    // set property value
    err = dcamprop_setvalue( hdcam, DCAM_IDPROP_SHUTTER_MODE, prop );
    if( failed(err) )
    {
        //printf( "Error: set DCAM_IDPROP_SHUTTER_MODE\n" );
        printCameraError(cameraIndex, hdcam, err, "dcamprop_setvalue(DCAM_IDPROP_SHUTTER_MODE)\n");
        ret = -1;
    }

    return ret;
}

/**
  * READOUT TIME (R/O) property returns frame read out time in seconds.
  */
int OrcaUsbDriver::getReadoutTime(double *value)
{
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
        //printf( "Error: get DCAM_IDPROP_TIMING_READOUTTIME\n" );
        printCameraError(cameraIndex, hdcam, err, "dcamprop_getvalue(DCAM_IDPROP_TIMING_READOUTTIME)\n");
        ret = -1;
    }

    return ret;
}

/**
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
  */
int OrcaUsbDriver::getTimingExposure(double *value)
{
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
        //printf( "Error: get DCAM_IDPROP_TIMING_EXPOSURE\n" );
        printCameraError(cameraIndex, hdcam, err, "dcamprop_getvalue(DCAM_IDPROP_TIMING_EXPOSURE)\n");
        ret = -1;
    }

    return ret;
}

/**
  * GLOBAL EXPOSURE DELAY (R/O) property returns how long GLOBAL EXPOSURE is delayed from beginning of EXPOSURE itself.
  * If the sensor does not have GLOBAL SHUTTER capability, GLOBAL EXPOSURE timing, 
  * which means all pixels on the sensor is exposed, is delayed. 
  *	This property is EFFECTIVE when DCAM_IDPROP_TRIGGER_GLOBAL_EXPOSURE is DELAYED.
  */
int OrcaUsbDriver::getGlobalExposureDelay(double *value)
{
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
        //printf( "Error: get DCAM_IDPROP_TIMING_GLOBALEXPOSUREDELAY\n" );
        printCameraError(cameraIndex, hdcam, err, "dcamprop_getvalue(DCAM_IDPROP_TIMING_GLOBALEXPOSUREDELAY)\n");
        ret = -1;
    }

    return ret;
}

/**
  * INVALID EXPOSURE PERIOD (R/O) value shows how long takes starting exposure from input trigger.
  * Because of its structure, some sensors cannot start exposure immediately.
  * There are various reasons but this property just tells how long it is.
  * This value does not include jitter of input trigger.
  */
int OrcaUsbDriver::getInvalidExposurePeriod(double *value)
{
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
        //printf( "Error: get DCAM_IDPROP_TIMING_INVALIDEXPOSUREPERIOD\n" );
        printCameraError(cameraIndex, hdcam, err, "dcamprop_getvalue(DCAM_IDPROP_TIMING_INVALIDEXPOSUREPERIOD)\n");
        ret = -1;
    }

    return ret;
}

/**
  * TRIGGER DELAY (R/W) property can set delay time for using this timing inside of camera.
  */
int OrcaUsbDriver::getTriggerDelay(double *value)
{
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
        //printf( "Error: get DCAM_IDPROP_TRIGGERDELAY\n" );
        printCameraError(cameraIndex, hdcam, err, "dcamprop_getvalue(DCAM_IDPROP_TRIGGERDELAY)\n");
        ret = -1;
    }

    return ret;
}

int OrcaUsbDriver::setTriggerDelay(double value)
{
    int ret = 0;
    DCAMERR err;

    // set trigger delay value in seconds
    err = dcamprop_setvalue( hdcam, DCAM_IDPROP_TRIGGERDELAY, value );
    if( failed(err) )
    {
        //printf( "Error: set DCAM_IDPROP_TRIGGERDELAY\n" );
        printCameraError(cameraIndex, hdcam, err, "dcamprop_setvalue(DCAM_IDPROP_TRIGGERDELAY)\n");
        ret = -1;
    }

    return ret;
}

/**
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
  */
int OrcaUsbDriver::getTriggerGlobalExposure(double *value)
{
    int ret = 0;
    DCAMERR err;
    double data;

    if(!value)
        return -1;

    err = dcamprop_getvalue( hdcam, DCAM_IDPROP_TRIGGER_GLOBALEXPOSURE, &data );
    if( !failed(err) )
    {
        *value = data;
        //printf( "#%d: RBV of DCAM_IDPROP_TRIGGER_GLOBALEXPOSURE: %d\n", cameraIndex, (int)data);
    }
    else
    {
        //printf( "Error: get DCAM_IDPROP_TRIGGER_GLOBALEXPOSURE\n" );
        printCameraError(cameraIndex, hdcam, err, "dcamprop_getvalue(DCAM_IDPROP_TRIGGER_GLOBALEXPOSURE)\n");
        ret = -1;
    }

    return ret;
}

int OrcaUsbDriver::setTriggerGlobalExposure(int value)
{
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
            printf( "#%d: Error: invalid value for DCAM_IDPROP_TRIGGER_GLOBALEXPOSURE: %d\n", cameraIndex, value );
            return -1;
    }

    // set property value
    err = dcamprop_setvalue( hdcam, DCAM_IDPROP_TRIGGER_GLOBALEXPOSURE, prop );
    if( failed(err) )
    {
        //printf( "Error: set DCAM_IDPROP_TRIGGER_GLOBALEXPOSURE with value: %d\n", value);
        printCameraError(cameraIndex, hdcam, err, "dcamprop_setvalue(DCAM_IDPROP_TRIGGER_GLOBALEXPOSURE)\n");
        ret = -1;
    }
    else
    {
        //printf( "#%d: DCAM_IDPROP_TRIGGER_GLOBALEXPOSURE set to %d\n", cameraIndex, value );
    }

    return ret;
}

int OrcaUsbDriver::setBinning(int binning)
{
    DCAMERR err;
    double value;

    err = dcamprop_queryvalue(hdcam, DCAM_IDPROP_BINNING, &value);

    if(failed(err))
    {
        printf("#%d: Error: dcamprop_queryvalue() IDPROP:BINNING, VALUE:%d\n", cameraIndex, binning);
        printCameraError(cameraIndex, hdcam, err, "dcamprop_queryvalue()\n");
        return -1;
    }

    err = dcamprop_setvalue(hdcam, DCAM_IDPROP_BINNING, binning);

    if(failed(err))
    {
        printf("#%d: Error: dcamprop_setvalue() IDPROP:BINNING, VALUE:%d\n", cameraIndex, binning);
        printCameraError(cameraIndex, hdcam, err, "dcamprop_value()\n");
        return -1;
    }

    return 0;
}

int OrcaUsbDriver::setSubarray(int hpos, int hsize, int vpos, int vsize)
{
    DCAMERR err;

    err = dcamprop_setvalue(hdcam, DCAM_IDPROP_SUBARRAYMODE, DCAMPROP_MODE__OFF);

    if(failed(err))
    {
        printf("#%d: Error: dcamprop_setvalue() IDPROP:SUBARRAYMODE, VALUE:OFF\n", cameraIndex);
        return -1;
    }

    err = dcamprop_setvalue(hdcam, DCAM_IDPROP_SUBARRAYHPOS, hpos);

    if(failed(err))
    {
        printf("#%d: Error: dcamprop_setvalue() IDPROP:SUBARRAYHPOS, VALUE:%d\n", cameraIndex, hpos);
        return -1;
    }

    err = dcamprop_setvalue(hdcam, DCAM_IDPROP_SUBARRAYHSIZE, hsize);

    if(failed(err))
    {
        printf("#%d: Error: dcamprop_setvalue() IDPROP:SUBARRAYHSIZE, VALUE:%d\n", cameraIndex, hsize);
        return -1;
    }

    err = dcamprop_setvalue(hdcam, DCAM_IDPROP_SUBARRAYVPOS, vpos);

    if(failed(err))
    {
        printf("#%d: Error: dcamprop_setvalue() IDPROP:SUBARRAYVPOS, VALUE:%d\n", cameraIndex, vpos);
        return -1;
    }

    err = dcamprop_setvalue(hdcam, DCAM_IDPROP_SUBARRAYVSIZE, vsize);

    if(failed(err))
    {
        printf("#%d: Error: dcamprop_setvalue() IDPROP:SUBARRAYVSIZE, VALUE:%d\n", cameraIndex, vsize);
        return -1;
    }

    err = dcamprop_setvalue(hdcam, DCAM_IDPROP_SUBARRAYMODE, DCAMPROP_MODE__ON);

    if(failed(err))
    {
        printf("#%d: Error: dcamprop_setvalue() IDPROP:SUBARRAYMODE, VALUE:ON\n", cameraIndex);
        return -1;
    }

    return 0;
}

int OrcaUsbDriver::getCamIndex()
{
    return cameraIndex;    
}

asynStatus OrcaUsbDriver::getGeometry()
{
    int status = asynSuccess;

    status |= getIntegerParam(ADBinX, &binX); if(binX<1) {binX =1; status |= setIntegerParam(ADBinX, binX);}
    status |= getIntegerParam(ADBinY, &binY); if(binY<1) {binY =1; status |= setIntegerParam(ADBinY, binY);}
    status |= getIntegerParam(ADMinX, &minX);
    status |= getIntegerParam(ADMinY, &minY);
    status |= getIntegerParam(ADSizeX, &sizeX);
    status |= getIntegerParam(ADSizeY, &sizeY);
    status |= getIntegerParam(ADMaxSizeX, &maxSizeX);
    status |= getIntegerParam(ADMaxSizeY, &maxSizeY);

    return asynStatus(status);
}

void OrcaUsbDriver::printCameraError(int camIndex, HDCAM hdcam, DCAMERR errid, const char* apiname)
{
        char errtext[ 256 ];
        DCAMERR err;
        DCAMDEV_STRING  param;

        memset( &param, 0, sizeof(param) );
        param.size      = sizeof(param);
        param.text      = errtext;
        param.textbytes = sizeof(errtext);
        param.iString   = errid;

        err = dcamdev_getstring( hdcam, &param );

        if( !failed(err) )
        {
            printf( "FAILED(#%d): 0x%08X: %s @ %s\n", camIndex, errid, errtext, apiname );
        }
        else
        {
            printf( "#%d: Error: getting error string for %s error\n", camIndex, apiname );
        }
}

//----------------------------------------------------------------------------
// OrcaUsb Constructor/Destructor
//----------------------------------------------------------------------------

/** Constructor for OrcaUsb driver; most parameters are simply passed to ADDriver::ADDriver.
  * After calling the base class constructor this method creates a thread to collect the detector data, 
  * and sets reasonable default values for the parameters defined in this class, asynNDArrayDriver and ADDriver.
  * \param[in]: The name of the asyn port driver to be created.
  * \param[in]: The ID (serial number) of the camera.
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
               priority, stackSize)
{
    char    str_message[128];
    double  tmpfloat64;
    int     tmpint32;
    int     cameraFound = 0;
    int     i;

    DCAMERR err;
    double value;

    //ellInit(&orcaList);           /* initialize linked list for camera information */
    initPerfMeasure();

    if(!initialized) 
    {
        /* first time to initialize */

        //initLock = epicsMutexCreate();  /* mutex lock for init */

        memset( &apiinit, 0, sizeof(apiinit) );
        apiinit.size = sizeof(apiinit);

        // initialize DCAM-API
        err = dcamapi_init( &apiinit );

        if( failed(err) )
        {
            printf( "Error: initializing API\n" );
            return;
        }

        for (i=0; i<MAX_CAM_NUM; i++)
            openCameras[i] = 0;

        initialized = 1;

        deviceCount = apiinit.iDeviceCount;
        printf( "Found %d device(s).\n", deviceCount );
    }

    // enumarate cameras and find the one by cameraId    
    hdcam = NULL;

    for(i=0; i<deviceCount; i++)
    {
        // open device
        DCAMDEV_OPEN devopen;
        memset( &devopen, 0, sizeof(devopen) );
        devopen.size = sizeof(devopen);
        devopen.index = i;

        // open and check unused camreas only
        if (openCameras[i] == 0)
        {
            printf( "#%d: Trying to open camera\n", i );
            err = dcamdev_open( &devopen );
            if( failed(err) )
            {
                printf( "#%d: Error: opening camera\n", i );
                return;
            }
            else
            {
                printf( "#%d: Opened for ID check\n", i );

                hdcam = devopen.hdcam;
                char data[256];

                DCAMDEV_STRING    param;
                memset( &param, 0, sizeof(param) );
                param.size = sizeof(param);
                param.text = data;
                param.textbytes = sizeof(data);

                param.iString = DCAM_IDSTR_MODEL;
                err = dcamdev_getstring( hdcam, &param );
                if( !failed(err) )
                {
                    printf( "#%d: Model: %s\n", i, data);
                }
                else
                {
                    printf( "#%d: Error: getting model\n", i );
                }

                param.iString = DCAM_IDSTR_BUS;
                err = dcamdev_getstring( hdcam, &param );
                if( !failed(err) )
                {
                    printf( "#%d: Bus: %s\n", i, data);
                }
                else
                {
                    printf( "#%d: Error: getting bus\n", i );
                }

                param.iString = DCAM_IDSTR_CAMERAID;
                err = dcamdev_getstring( hdcam, &param );
                if( !failed(err) )
                {
                    printf( "#%d: ID: %s\n", i, data);
                    // check camera serial number
                    if( strstr(data, cameraId) )
                    {
                        // camera with specified serial number found
                        cameraFound = 1;
                        openCameras[i] = 1;
                        cameraIndex = i;

                        printf( "#%d: Camera with ID: %s found\n", i, cameraId );

                        if (getSensorMode(&value) != -1)
                            printf( "Sensor Mode: %d\n", (int)value);

                        // shutter mode property doesn't exists
                        //if (getShutterMode(&value) != -1)
                        //    printf( "Shutter Mode: %f\n", value);
                        //setShutterMode(DCAMPROP_SHUTTER_MODE__GLOBAL);

                        //set global exposure to GLOBAL RESET (default is DELAYED)
                        setTriggerGlobalExposure(DCAMPROP_TRIGGER_GLOBALEXPOSURE__GLOBALRESET);

                        if (getTriggerGlobalExposure(&value) != -1)
                            printf( "Trigger Global Exposure: %d\n", (int)value);

                        if (getTimingExposure(&value) != -1)
                            printf( "Timing Exposure: %d\n", (int)value);

                        if (getGlobalExposureDelay(&value) != -1)
                            printf( "Exposure Delay (ms): %f\n", 1000*value);

                        if (getInvalidExposurePeriod(&value) != -1)
                            printf( "Invalid Exposure Period (ms): %f\n", 1000*value);

                        if (getTriggerDelay(&value) != -1)
                            printf( "Trigger Delay (ms): %f\n", 1000*value);

                        //setTriggerDelay(0.02);
                        //if (getTriggerDelay(&value) != -1)
                        //    printf( "Trigger Delay after (ms): %f\n", 1000*value);

                        if (getReadoutTime(&value) != -1)
                            printf( "Readout Time (ms): %f\n", 1000*value);

                        break;
                    }
                    else
                    {
                        // close camera with unmatching serial number 
                        dcamdev_close( hdcam );
                        hdcam = NULL;
                        printf( "#%d: Unmatching camera closed\n", i );
                    }
                }
                else
                {
                    printf( "#%d: Error: getting ID\n", i );

                    // close camera if serial number could not be retreived
                    dcamdev_close( hdcam );
                    hdcam = NULL;
                    printf( "#%d: Unidentified camera closed\n", i );
                }
            }
        }
        else
        {
            printf( "#%d: Camera already open\n", i );
        }
    }

    if (!cameraFound)
    {
        printf( "Error: no unopen camera with ID: %s found\n", cameraId );
        if (hdcam != NULL)
        {
            dcamdev_close( hdcam );
            hdcam = NULL;
        }

        return;
    }

    //serialLock = epicsMutexMustCreate();
    dataEvent  = epicsEventMustCreate(epicsEventEmpty);

    // Create PV Params
    createParam(OrcaCameraNameString,               asynParamOctet, &cameraName);
    createParam(OrcaCameraSerialString,             asynParamOctet, &cameraSerial);
    createParam(OrcaCameraFirmwareString,           asynParamOctet, &cameraFirmware);
    createParam(OrcaCameraInfoString,               asynParamOctet, &cameraInfo);
    createParam(OrcaCameraTrgString,                asynParamInt32, &cameraTrg);
    createParam(OrcaCameraTrgPolarityString,        asynParamInt32, &cameraTrgPolarity);
    createParam(OrcaCameraTrgGlobalExposureString,  asynParamInt32, &cameraTrgGlobalExposure);

    createParam(OrcaCameraImgProcTimeString,   asynParamFloat64, &cameraImageProcTime);
    createParam(OrcaCameraCBProcTimeString,    asynParamFloat64, &cameraCallbackProcTime);
    createParam(OrcaCameraWaitProcTimeString,  asynParamFloat64, &cameraWaitProcTime);
    createParam(OrcaCameraStartProcTimeString, asynParamFloat64, &cameraStartProcTime);
    createParam(OrcaCameraEllapseTimeString,   asynParamFloat64, &cameraEllapseTime);

    createParam(OrcaCameraLostFrameSyncString, asynParamInt32, &cameraLostFrameSyncCounter);

    acquire = 0;
    exit_loop = 0;

    setStringParam(ADManufacturer, "Hamamatsu");

    if(!getCameraName(str_message))    
    {
        setStringParam(cameraName, str_message);  
        setStringParam(ADModel, str_message); 
    }

    if(!getCameraSerial(str_message)) 
        setStringParam(cameraSerial, str_message);

    if(!getCameraFirmware(str_message)) 
        setStringParam(cameraFirmware, str_message);

    if(!getCameraInfo(str_message))
        setStringParam(cameraInfo, str_message);

    setIntegerParam(cameraTrg, 0);         /* Edge Trigger */
    setIntegerParam(cameraTrgPolarity, 0); /* Negative Polarity */

    //setIntegerParam(cameraTrgGlobalExposure, 5); /* Global Reset */

    if(!getActualExposure(&tmpfloat64))
        setDoubleParam(ADAcquireTime, tmpfloat64);

    if(!getEffectiveSizeX(&tmpint32))
        setIntegerParam(ADMaxSizeX, tmpint32);

    if(!getEffectiveSizeY(&tmpint32))
        setIntegerParam(ADMaxSizeY, tmpint32);

    // register shutdown function for epicsAtExit
    epicsAtExit(exitHookC, this);

    // create new thread (it terminates when funtion ptr returns)
    epicsThreadCreate("OrcaTask",                      /* name */
                       epicsThreadPriorityMedium,      /* priority */
                       stackSize,                      /* stack size */
                       (EPICSTHREADFUNC) dataTaskC,    /* funtion pointer */
                       this);                          /* argument passed */ 
}

/*
 * OrcaUsb destructor. Called by the exitHookC function when IOC is shut down
 */
OrcaUsbDriver::~OrcaUsbDriver()
{
    printf( "IOC shutdown\n" );

    // close device
    if (hdcam != NULL)
    {
        dcamdev_close( hdcam );
        printf( "Camera %d closed on exit\n", cameraIndex );
    }
}

//-------------------------------------------------------------------------
// ADDriver function overwrites
//-------------------------------------------------------------------------

/*
 * Function overwriting ADDriver base function.
 * Takes in a function (PV) changes, and a value it is changing to, and processes the input
 *
 * \params[in]: asyn client who requests a write
 * \params[in]: int32 value to write
 * \return: success if write was successful, else failure
 */
asynStatus OrcaUsbDriver::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
    static const char *functionName = "writeInt32";
    printf("#%d: (%s) function: %d, value: %d\n", cameraIndex, functionName, pasynUser->reason, value);

    if(pasynUser->reason == ADAcquire) 
    {
        if(value && !acquire) 
        {
            acquire = 1;
            setIntegerParam(ADNumImagesCounter, 0);
            epicsEventSignal(dataEvent);
        }

        if(!value && acquire) 
        {
            acquire = 0;
        }
    }

    if(pasynUser->reason == ADMinX)  { setIntegerParam(ADMinX, value); }
    if(pasynUser->reason == ADMinY)  { setIntegerParam(ADMinY, value); }
    if(pasynUser->reason == ADSizeX) { setIntegerParam(ADSizeX, value); }
    if(pasynUser->reason == ADSizeY) { setIntegerParam(ADSizeY, value); }


    if(pasynUser->reason == ADImageMode)    setIntegerParam(ADImageMode,    value);
    if(pasynUser->reason == ADNumImages)    setIntegerParam(ADNumImages,    value);
    if(pasynUser->reason == NDArrayCounter) setIntegerParam(NDArrayCounter, value);

    if(pasynUser->reason == ADTriggerMode)
        if(!setExposureControl(value)) setIntegerParam(ADTriggerMode, value);

    if(pasynUser->reason == cameraTrg)
        if(!setTriggerControl(value)) setIntegerParam(cameraTrg, value);

    if(pasynUser->reason == cameraTrgPolarity)
        if(!setTriggerPolarity(value)) setIntegerParam(cameraTrgPolarity, value);

    if(pasynUser->reason == cameraTrgGlobalExposure)
        if(!setTriggerGlobalExposure(value)) setIntegerParam(cameraTrgGlobalExposure, value); 

    callParamCallbacks();

    return asynStatus(0);
}

/*
 * Function overwriting ADDriver base function.
 * Takes in a function (PV) changes, and a value it is changing to, and processes the input
 * This is the same functionality as writeInt32, but for processing doubles.
 *
 * \params[in]: asyn client who requests a write
 * \params[in]: float value to write
 * \return: success if write was successful, else failure
 */
asynStatus OrcaUsbDriver::writeFloat64(asynUser *pasynUser, epicsFloat64 value)
{
    static const char *functionName = "writeFloat64";
    printf("#%d: (%s) function: %d, value %lf\n", cameraIndex, functionName, pasynUser->reason, value);

    if(pasynUser->reason == ADAcquireTime) 
    {
        double tmpfloat64;
        setExposure(value);
        if(!getActualExposure(&tmpfloat64)) setDoubleParam(ADAcquireTime, tmpfloat64);
    }

    callParamCallbacks();

    return asynStatus(0);
}

void OrcaUsbDriver::dataTask(void)
{
    DCAMERR err;

    int ndims = 2;
    size_t dims[2] = {2048, 2048};
    NDArray *pArray;
    unsigned short *pData = NULL;
    int width, height;

    int imageCounter;
    //unsigned evr_ticks;
    //int frame_sync;
    //u_int frame_counter;
    //int lostFrameSyncCounter =0;

    //int check_c = 0;

    DCAMWAIT_OPEN waitopen;
    HDCAMWAIT hwait;

    perfParm_ts *perf_wait     = makePerfMeasure((char*)"WAIT", (char*)"Wait Image");
    perfParm_ts *perf_start    = makePerfMeasure((char*)"START", (char*)"Start Image");
    perfParm_ts *perf_proc     = makePerfMeasure((char*)"PROCESS", (char*)"Process Image");
    perfParm_ts *perf_callback = makePerfMeasure((char*)"CALLBACK", (char*)"Callback Only");

    printf("#%d: Start Acquire Thread\n", cameraIndex);

    memset( &waitopen, 0, sizeof(waitopen) );
    waitopen.size  = sizeof(waitopen);
    waitopen.hdcam = hdcam;

    err = dcamwait_open( &waitopen );
    if( failed(err) )
    {
        //printf("#%d: Error: dcamwait_open()\n", cameraIndex);
        printCameraError(cameraIndex, hdcam, err, "dcamwait_open()\n");
        //return;
    }

    hwait = waitopen.hwait;

    err = dcambuf_alloc( hdcam, 1 );
    if( failed(err) )
    {
        //printf("#%d: Error: dcambuf_alloc()\n", cameraIndex);
        printCameraError(cameraIndex, hdcam, err, "dcambuf_alloc()\n");
        //return;
    }

    while(!exit_loop) /* infinte loop */
    {  
        printf("#%d: Waiting for event...\n", cameraIndex);
        epicsEventWait(dataEvent);
        printf("#%d: Event arrived\n", cameraIndex);

        if (exit_loop) break;

        {  /* prepare the imageMode */
            int imageMode, numImages;
            getIntegerParam(ADImageMode, &imageMode);
            getIntegerParam(ADNumImages, &numImages);

            switch(imageMode) 
            {
                case ADImageSingle:
                    this->framesRemaining = 1;
                break;
                case ADImageMultiple:
                    this->framesRemaining = numImages;
                break;
                case ADImageContinuous:
                    this->framesRemaining = -1;
                break;
            }
        }

        setIntegerParam(ADStatus, ADStatusAcquire);
        printf("#%d: Start Acquire Loop\n", cameraIndex);

        getGeometry();

        //setSubarray(minX, sizeX, minY, sizeY);

        pData = new unsigned short[sizeX*sizeY];

        width = sizeX;
        height = sizeY;

        dims[0] = width; dims[1] = height;

        setIntegerParam(NDDataType, NDUInt16);
        setIntegerParam(NDArrayCallbacks, 1);
        setIntegerParam(NDArraySizeX, width);
        setIntegerParam(NDArraySizeY, height);
        setIntegerParam(NDArraySize, width*height);

        // reset counters
        setIntegerParam(ADNumImagesCounter, 0);
        setIntegerParam(ADNumExposuresCounter, 0);
        callParamCallbacks();

        err = dcamcap_start( hdcam, DCAMCAP_START_SEQUENCE );
        if( failed(err) )
        {
            //printf("#%d: Error: dcamcap_start()\n", cameraIndex);
            printCameraError(cameraIndex, hdcam, err, "dcamcap_start()\n");
            //return;
        }

        while(acquire) 
        {
            startPerfMeasure(perf_wait);

            DCAMWAIT_START waitstart;
            memset( &waitstart, 0, sizeof(waitstart) );
            waitstart.size      = sizeof(waitstart);
            waitstart.eventmask = DCAMWAIT_CAPEVENT_FRAMEREADY;
            waitstart.timeout   = 1000;

            err = dcamwait_start( hwait, &waitstart );
            if( failed(err) )
            {
                //printf("#%d: Error: dcamwait_start()\n", cameraIndex);
                printCameraError(cameraIndex, hdcam, err, "dcamwait_start()\n");
                continue;
            }

            DCAMCAP_TRANSFERINFO captransferinfo;
            memset( &captransferinfo, 0, sizeof(captransferinfo) );
            captransferinfo.size    = sizeof(captransferinfo);

            err = dcamcap_transferinfo( hdcam, &captransferinfo );
            if( failed(err) )
            {
                //printf("#%d: Error: dcamcap_transferinfo()\n", cameraIndex);
                printCameraError(cameraIndex, hdcam, err, "dcamcap_transferinfo()\n");
                continue;
            }

            if( captransferinfo.nFrameCount < 1 )
            {
                printf("#%d: Error: no image captured\n", cameraIndex);
                continue;
            }
            else
            {
                printf("#%d: got %d images\n", cameraIndex, captransferinfo.nFrameCount);
            }

            endPerfMeasure(perf_wait);

            if(!acquire) continue;

            DCAMBUF_FRAME bufframe;
            memset( &bufframe, 0, sizeof(bufframe) );
            bufframe.size = sizeof(bufframe);
            bufframe.iFrame = 0;

            int rowbytes;
            getRowBytes(&rowbytes);

            bufframe.buf      = pData;
            bufframe.rowbytes = rowbytes;
            bufframe.left     = minX;
            bufframe.top      = minY;
            bufframe.width    = width;
            bufframe.height   = height;

            // access image
            err = dcambuf_copyframe( hdcam, &bufframe );
            if( failed(err) )
            {
                //printf("#%d: Error: dcambuf_copyframe()\n", cameraIndex);
                printCameraError(cameraIndex, hdcam, err, "dcambuf_copyframe()\n");
            }

            //startPerfMeasure(perf_start);
            //endPerfMeasure(perf_start);

            startPerfMeasure(perf_proc);
            
            this->lock();
            pArray = this->pNDArrayPool->alloc(ndims, dims, NDUInt16, 0, NULL);
            pArray->ndims=2;
            pArray->dims[0].size    = width;
            pArray->dims[0].offset  = minX;
            pArray->dims[0].binning = binX;
            pArray->dims[1].size    = height;
            pArray->dims[1].offset  = minY;
            pArray->dims[1].binning = binY;
            updateTimeStamp(&pArray->epicsTS);
            pArray->uniqueId = PULSEID(pArray->epicsTS);

            memcpy(pArray->pData, pData, width*height*sizeof(unsigned short));

            {   /* provide POSIX timestamp for the image native timestamp */
                timespec ts;
                epicsTimeToTimespec(&ts, &pArray->epicsTS);
                pArray->timeStamp = (double)ts.tv_sec + ((double)ts.tv_nsec * 1.0e-09);
            }

            //ErGetTicks(0, &evr_ticks); setDoubleParam(cameraEllapseTime, double(evr_ticks)/119.0);
            this->unlock();
            
            startPerfMeasure(perf_callback);
            
            doCallbacksGenericPointer(pArray, NDArrayData, 0);

            endPerfMeasure(perf_callback);
            
            this->lock();
            pArray->release();

            if(this->framesRemaining > 0) this->framesRemaining--;
            if(this->framesRemaining == 0) 
            {
                setIntegerParam(ADAcquire, 0);
                acquire = 0;
            }

            getIntegerParam(ADNumImagesCounter, &imageCounter);
            imageCounter++;
            setIntegerParam(ADNumImagesCounter, imageCounter);

            getIntegerParam(NDArrayCounter, &imageCounter);
            imageCounter++;
            setIntegerParam(NDArrayCounter, imageCounter);
            
            endPerfMeasure(perf_proc);

            calcPerfMeasure(perf_wait);      setDoubleParam(cameraWaitProcTime, perf_wait->elapsed_time);
            calcPerfMeasure(perf_start);     setDoubleParam(cameraStartProcTime, perf_start->elapsed_time);
            calcPerfMeasure(perf_callback);  setDoubleParam(cameraCallbackProcTime, perf_callback->elapsed_time);
            calcPerfMeasure(perf_proc);      setDoubleParam(cameraImageProcTime, perf_proc->elapsed_time);

            callParamCallbacks();
            this->unlock();
        }

        dcamcap_stop( hdcam );
        
        setIntegerParam(ADStatus, ADStatusIdle);
        callParamCallbacks();
        printf("#%d: Stop Acquire Loop\n", cameraIndex);
    } /* end of infinite loop */

    delete[] pData;

    dcambuf_release( hdcam );
    dcamwait_close( hwait );
    printf("#%d: Terminate Acquire Thread\n", cameraIndex);
}

void OrcaUsbDriver::exitHook(void)
{
    printf( "Exit loop of camera %d called\n", cameraIndex );

    if (hdcam != NULL)
    {
        dcamdev_close( hdcam );
        printf( "Camera %d closed on exit\n", cameraIndex );
    }

    exit_loop = 1;
    acquire = 0;
    
    epicsEventSignal(dataEvent);

    epicsThreadSleep(2.);
}

/**
 * External configuration function for OrcaUsb.
 * Envokes the constructor to create a new OrcaUsb object
 * This is the function that initializes the driver, and is called in the IOC startup script
 *
 * \params[in]: all passed into constructor
 * \return: status
 */extern "C" {
int OrcaConfig(char *portName, char *cameraId, int maxBuffers, size_t maxMemory, int priority, int stackSize)
{
    new OrcaUsbDriver(portName, cameraId, maxBuffers, maxMemory, priority, stackSize);
    return(asynSuccess);
}

/* Code for iocsh registration */
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
static void  OrcaConfigCallFunc(const iocshArgBuf *args)
{
    OrcaConfig(args[0].sval, args[1].sval, args[2].ival, 
               args[3].ival, args[4].ival, args[5].ival);
}

static void OrcaConfigRegister(void)
{
    iocshRegister(&OrcaConfigFuncDef, OrcaConfigCallFunc);
}


epicsExportRegistrar(OrcaConfigRegister);
}

