TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE

#======== DETECTOR LIBRARY ==============

USR_CXXFLAGS_Linux += -D_LINUX -D_x86

LIBRARY_IOC_Linux += orcaUsbSupport
LIB_SRCS += orcaUsbDriver.cpp

#LIB_LIBS += dcamapi
#LIB_LIBS += asyn
#LIB_LIBS += perfMeasure

DBD += orcaUsbDriver.dbd

include $(ADCORE)/ADApp/commonLibraryMakefile

#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

