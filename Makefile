
include $(VMLROOT)/lib/vmlcommon.mk

INCLUDES+=$(BVL_INCLUDE) $(VML_INCLUDE) $(OPTO_INCLUDE) $(TCL_INCLUDE)

CXXFLAGS += /MDd /GR /RTC1 /EHs /Zi /Fd"vc90.pdb" /D"COIN_DLL"
LINKARGS += /DEBUG /PDB:"info.pdb"

LIBS=$(BVL_LIB) $(VML_LIB) $(TCL_LIB) $(COIN3D_LIB) $(OPTO_LIB)
LIBS += user32.lib opengl32.lib glu32.lib gdi32.lib 


SRCS:= CalibrationManager.cc Calibrator.cc EM.cc PlaneFit.cc Point5D.cc ScreenPointGenerator.cc ViewerCalib.cc main.cpp State.cc Render.cc RobustCalibrator.cc
OBJS:=$(patsubst %.cpp, %.obj, $(filter %.cpp, $(SRCS))) \
	$(patsubst %.cc, %.obj, $(filter %.cc, $(SRCS)))

TARGET=cubycalib

all: $(TARGET) 

cubycalib.res: cubycalib.rc
	rc /r cubycalib.rc

$(TARGET): $(OBJS) cubycalib.res
	$(LINK) $(LINKARGS) $^ $(LIBS) 
	mt -nologo -manifest $(TARGET).exe.manifest -outputresource:$(TARGET).exe\;1

tplanefit: tplanefit.o PlaneFit.o Calibrator.o Point5D.o
	$(LINK) $(LINKARGS) $^ $(BVL_LIB) $(VML_LIB) 

filecalib: filecalib.o EM.o Calibrator.o Point5D.o PlaneFit.o  RobustCalibrator.o
	$(LINK) $(LINKARGS) $^ $(BVL_LIB) $(VML_LIB) 

simulate: simulate.o EM.o Calibrator.o Point5D.o PlaneFit.o ScreenPointGenerator.o
	$(LINK) $(LINKARGS) $^ $(BVL_LIB) $(OPTPP_LIB) $(VML_LIB) 

sidist: sidist.o EM.o Calibrator.o Point5D.o PlaneFit.o ScreenPointGenerator.o
	$(LINK) $(LINKARGS) $^ $(BVL_LIB) $(OPTPP_LIB) $(VML_LIB) 

mcsi: mcsi.o EM.o Calibrator.o Point5D.o PlaneFit.o ScreenPointGenerator.o
	$(LINK) $(LINKARGS) $^ $(BVL_LIB) $(OPTPP_LIB) $(VML_LIB) 

.PHONY: clean depend

depend:
	makedepend -Y -- $(SRCS)

clean:
	rm -f $(TARGET) $(OBJS)

