#ifndef ORCAUSBDRV_H
#define ORCAUSBDRV_H

#include "ADDriver.h"
#include "dcamapi4.h"
#include "dcamprop.h"


#define MAX_CAM_NUM 10

#define OrcaCameraNameString                "CAMERA_NAME"
#define OrcaCameraSerialString              "CAMERA_SERIAL"
#define OrcaCameraFirmwareString            "CAMERA_FIRMWARE"
#define OrcaCameraInfoString                "CAMERA_INFO"
#define OrcaCameraTrgString                 "CAMERA_TRG"
#define OrcaCameraTrgPolarityString         "CAMERA_TRGPOL"
#define OrcaCameraTrgGlobalExposureString   "CAMERA_TRGGLOBALEXP"
#define OrcaCameraReadStatString            "CAMERA_READ_STAT"
#define OrcaCameraStatusString              "CAMERA_STATUS"


static int              initCounter = 0;
static DCAMAPI_INIT     apiinit;
static int              deviceCount = 0;
static int              openCameras[MAX_CAM_NUM];


class OrcaUsbDriver : public ADDriver {
    public:
        OrcaUsbDriver(const char *portName, const char* cameraId, int maxBuffers, size_t maxMemory,
                   int priority, int stackSize);
        virtual ~OrcaUsbDriver();
        virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
        virtual asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
        virtual void report(FILE *fp, int details);

        void dataTask(void);
        void exitHook(void);
        int getCamIndex(void);

    protected:
        epicsEventId dataEvent;

        int cameraName;
        #define FIRST_ORCA_PARAM cameraName
        int cameraSerial;
        int cameraFirmware;
        int cameraInfo;
        int cameraTrg;
        int cameraTrgPolarity;
        int cameraTrgGlobalExposure;
        int cameraReadStat;
        int cameraStatus;
        #define LAST_ORCA_PARAM cameraStatus
        #define NUM_ORCA_DET_PARAMS ((int)(&LAST_ORCA_PARAM - &FIRST_ORCA_PARAM + 1))

    private:
        std::string port_name;
        bool acquire;
        bool exit_loop, exited;
        int binX, binY, minX, minY, sizeX, sizeY, maxSizeX, maxSizeY;

        HDCAM hdcam;
        int framesRemaining;
        int cameraIndex;
        int cameraAvailable;
        int findCamera(const char* cameraId);
        int getCameraName(char *value);
        int getCameraSerial(char *value);
        int getCameraFirmware(char *value);
        int getCameraInfo(char *value);
        int getCameraStatus();
        int getActualExposure(double *value);
        int getEffectiveSizeX(int *value);
        int getEffectiveSizeY(int *value);
        int getRowBytes(int *value);
        int setBinning(int binning);
        int getPixelType(int *value);
        int setExposure(double value);
        int setExposureControl(int value);
        int setTriggerControl(int value);
        int setTriggerPolarity(int value);
        int getSensorMode(double *value);
        int setSensorMode(int value);
        int getShutterMode(double *value);
        int setShutterMode(int value);
        int getReadoutTime(double *value);
        int getTimingExposure(double *value);
        int getGlobalExposureDelay(double *value);
        int getInvalidExposurePeriod(double *value);
        int getTriggerDelay(double *value);
        int setTriggerDelay(double value);
        int getTriggerGlobalExposure(double *value);
        int setTriggerGlobalExposure(int value);
        asynStatus configureROI();
        int getROI(int *hpos, int *hsize, int *vpos, int *vsize);
        int setROI(int hpos, int hsize, int vpos, int vsize);
        void printCameraError(int camIndex, HDCAM hdcam, DCAMERR errid, const char* apiname);

};

#endif
