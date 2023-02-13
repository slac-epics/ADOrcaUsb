#ifndef ORCAUSBDRV_H
#define ORCAUSBDRV_H

#define MAX_CAM_NUM 10

#include "ADDriver.h"
#include "dcamapi4.h"
#include "dcamprop.h"

class OrcaUsbDriver : public ADDriver {
    private:
        int acquire;
        int exit_loop;
        int binX, binY, minX, minY, sizeX, sizeY, maxSizeX, maxSizeY;

        int framesRemaining;

        HDCAM hdcam;
        int cameraIndex;
        int cameraAvailable;

        int findCameraById(const char* cameraId);
        int findCamera();
        int getCameraName(char *value);
        int getCameraSerial(char *value);
        int getCameraFirmware(char *value);
        int getCameraInfo(char *value);
        int getCameraStatus();
        int getActualExposure(double *value);
        int getEffectiveSizeX(int *value);
        int getEffectiveSizeY(int *value);
        int getRowBytes(int *value);
        //int getBinning(int *value);
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
        int getSubarray(int *hpos, int *hsize, int *vpos, int *vsize);
        int setSubarray(int hpos, int hsize, int vpos, int vsize);
        int setHPos(int hpos);
        int setHSize(int hsize);
        int setVPos(int vpos);
        int setVSize(int vsize);
        void printCameraError(int camIndex, HDCAM hdcam, DCAMERR errid, const char* apiname);

        asynStatus setGeometry();
        asynStatus getGeometry();
    public:
        OrcaUsbDriver(const char *portName, const char* cameraId, int maxBuffers, size_t maxMemory,
                   int priority, int stackSize);
        virtual ~OrcaUsbDriver();
        virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
        virtual asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);

        void dataTask(void);
        void exitHook(void);
        int getCamIndex(void);

    protected:
        //PdvDev *pdv_p;
        //EdtDev *edt_p;

        //epicsMutexId serialLock;
        epicsEventId dataEvent;

        int firstOrcaParam;
        #define FIRST_ORCA_PARAM firstOrcaParam
        int cameraName;
        int cameraSerial;
        int cameraFirmware;
        int cameraInfo;
        int cameraTrg;
        int cameraTrgPolarity;
        int cameraTrgGlobalExposure;
        int cameraReadStat;
        int cameraStatus;

        int cameraImageProcTime;
        int cameraCallbackProcTime;
        int cameraWaitProcTime;
        int cameraStartProcTime; 
        int cameraEllapseTime;

        int cameraLostFrameSyncCounter;

        int lastOrcaParam;
        #define LAST_ORCA_PARAM  lastOrcaParam

};

#define NUM_ORCA_DET_PARAMS ((int)(&LAST_ORCA_PARAM - &FIRST_ORCA_PARAM + 1))

#define OrcaCameraNameString                "CAMERA_NAME"
#define OrcaCameraSerialString              "CAMERA_SERIAL"
#define OrcaCameraFirmwareString            "CAMERA_FIRMWARE"
#define OrcaCameraInfoString                "CAMERA_INFO"
#define OrcaCameraTrgString                 "CAMERA_TRG"
#define OrcaCameraTrgPolarityString         "CAMERA_TRGPOL"
#define OrcaCameraTrgGlobalExposureString   "CAMERA_TRGGLOBALEXP"
#define OrcaCameraReadStatString            "CAMERA_READ_STAT"
#define OrcaCameraStatusString              "CAMERA_STATUS"

/* deugging */
#define OrcaCameraImgProcTimeString         "CAMERA_IMGPROCTIME"
#define OrcaCameraCBProcTimeString          "CAMERA_CBPROCTIME"
#define OrcaCameraWaitProcTimeString        "CAMERA_WAITPROCTIME"
#define OrcaCameraStartProcTimeString       "CAMERA_STARTPROCTIME"
#define OrcaCameraEllapseTimeString         "CAMERA_ELLAPSETIME"

#define OrcaCameraLostFrameSyncString       "CAMERA_LOSTFRAMESYNC"


//static ELLLIST          orcaList;
//static epicsMutexId     initLock = NULL;
static int              initCounter = 0;
static DCAMAPI_INIT     apiinit;
static int              deviceCount = 0;
static int              openCameras[MAX_CAM_NUM];

#endif
