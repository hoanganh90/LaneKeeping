#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Environment
MKDIR=mkdir
CP=cp
GREP=grep
NM=nm
CCADMIN=CCadmin
RANLIB=ranlib
CC=gcc
CCC=g++
CXX=g++
FC=gfortran
AS=as

# Macros
CND_PLATFORM=GNU-Linux
CND_DLIB_EXT=so
CND_CONF=Debug
CND_DISTDIR=dist
CND_BUILDDIR=build

# Include project Makefile
include Makefile

# Object Directory
OBJECTDIR=${CND_BUILDDIR}/${CND_CONF}/${CND_PLATFORM}

# Object Files
OBJECTFILES= \
	${OBJECTDIR}/Button.o \
	${OBJECTDIR}/Control.o \
	${OBJECTDIR}/LaneInfo.o \
	${OBJECTDIR}/LineSegment.o \
	${OBJECTDIR}/MSAC.o \
	${OBJECTDIR}/MySDL.o \
	${OBJECTDIR}/UDP_Connection.o \
	${OBJECTDIR}/Vision.o \
	${OBJECTDIR}/lmmin.o \
	${OBJECTDIR}/mavlink_control.o \
	${OBJECTDIR}/os.o


# C Compiler Flags
CFLAGS=-l raspicam -lraspicam_cv

# CC Compiler Flags
CCFLAGS=
CXXFLAGS=

# Fortran Compiler Flags
FFLAGS=

# Assembler Flags
ASFLAGS=

# Link Libraries and Options
LDLIBSOPTIONS=-lpthread `pkg-config --libs python-2.7` `pkg-config --libs SDL_image` `pkg-config --libs sdl` `pkg-config --libs libavcodec` `pkg-config --libs libavformat` `pkg-config --libs libavutil` `pkg-config --libs libswscale` /usr/local/lib/libopencv_core.so /usr/local/lib/libopencv_highgui.so /usr/local/lib/libopencv_imgcodecs.so /usr/local/lib/libopencv_imgproc.so /usr/local/lib/libopencv_videoio.so  

# Build Targets
.build-conf: ${BUILD_SUBPROJECTS}
	"${MAKE}"  -f nbproject/Makefile-${CND_CONF}.mk ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/lanekeeping1213

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/lanekeeping1213: /usr/local/lib/libopencv_core.so

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/lanekeeping1213: /usr/local/lib/libopencv_highgui.so

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/lanekeeping1213: /usr/local/lib/libopencv_imgcodecs.so

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/lanekeeping1213: /usr/local/lib/libopencv_imgproc.so

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/lanekeeping1213: /usr/local/lib/libopencv_videoio.so

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/lanekeeping1213: ${OBJECTFILES}
	${MKDIR} -p ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}
	${LINK.cc} -o ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/lanekeeping1213 ${OBJECTFILES} ${LDLIBSOPTIONS} -lX11

${OBJECTDIR}/Button.o: Button.cpp
	${MKDIR} -p ${OBJECTDIR}
	${RM} "$@.d"
	$(COMPILE.cc) -g `pkg-config --cflags python-2.7` `pkg-config --cflags SDL_image` `pkg-config --cflags sdl` `pkg-config --cflags libavcodec` `pkg-config --cflags libavformat` `pkg-config --cflags libavutil` `pkg-config --cflags libswscale` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/Button.o Button.cpp

${OBJECTDIR}/Control.o: Control.cpp
	${MKDIR} -p ${OBJECTDIR}
	${RM} "$@.d"
	$(COMPILE.cc) -g `pkg-config --cflags python-2.7` `pkg-config --cflags SDL_image` `pkg-config --cflags sdl` `pkg-config --cflags libavcodec` `pkg-config --cflags libavformat` `pkg-config --cflags libavutil` `pkg-config --cflags libswscale` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/Control.o Control.cpp

${OBJECTDIR}/LaneInfo.o: LaneInfo.cpp
	${MKDIR} -p ${OBJECTDIR}
	${RM} "$@.d"
	$(COMPILE.cc) -g `pkg-config --cflags python-2.7` `pkg-config --cflags SDL_image` `pkg-config --cflags sdl` `pkg-config --cflags libavcodec` `pkg-config --cflags libavformat` `pkg-config --cflags libavutil` `pkg-config --cflags libswscale` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/LaneInfo.o LaneInfo.cpp

${OBJECTDIR}/LineSegment.o: LineSegment.cpp
	${MKDIR} -p ${OBJECTDIR}
	${RM} "$@.d"
	$(COMPILE.cc) -g `pkg-config --cflags python-2.7` `pkg-config --cflags SDL_image` `pkg-config --cflags sdl` `pkg-config --cflags libavcodec` `pkg-config --cflags libavformat` `pkg-config --cflags libavutil` `pkg-config --cflags libswscale` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/LineSegment.o LineSegment.cpp

${OBJECTDIR}/MSAC.o: MSAC.cpp
	${MKDIR} -p ${OBJECTDIR}
	${RM} "$@.d"
	$(COMPILE.cc) -g `pkg-config --cflags python-2.7` `pkg-config --cflags SDL_image` `pkg-config --cflags sdl` `pkg-config --cflags libavcodec` `pkg-config --cflags libavformat` `pkg-config --cflags libavutil` `pkg-config --cflags libswscale` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/MSAC.o MSAC.cpp

${OBJECTDIR}/MySDL.o: MySDL.cpp
	${MKDIR} -p ${OBJECTDIR}
	${RM} "$@.d"
	$(COMPILE.cc) -g `pkg-config --cflags python-2.7` `pkg-config --cflags SDL_image` `pkg-config --cflags sdl` `pkg-config --cflags libavcodec` `pkg-config --cflags libavformat` `pkg-config --cflags libavutil` `pkg-config --cflags libswscale` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/MySDL.o MySDL.cpp

${OBJECTDIR}/UDP_Connection.o: UDP_Connection.cpp
	${MKDIR} -p ${OBJECTDIR}
	${RM} "$@.d"
	$(COMPILE.cc) -g `pkg-config --cflags python-2.7` `pkg-config --cflags SDL_image` `pkg-config --cflags sdl` `pkg-config --cflags libavcodec` `pkg-config --cflags libavformat` `pkg-config --cflags libavutil` `pkg-config --cflags libswscale` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/UDP_Connection.o UDP_Connection.cpp

${OBJECTDIR}/Vision.o: Vision.cpp
	${MKDIR} -p ${OBJECTDIR}
	${RM} "$@.d"
	$(COMPILE.cc) -g `pkg-config --cflags python-2.7` `pkg-config --cflags SDL_image` `pkg-config --cflags sdl` `pkg-config --cflags libavcodec` `pkg-config --cflags libavformat` `pkg-config --cflags libavutil` `pkg-config --cflags libswscale` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/Vision.o Vision.cpp

${OBJECTDIR}/lmmin.o: lmmin.c
	${MKDIR} -p ${OBJECTDIR}
	${RM} "$@.d"
	$(COMPILE.c) -g -Imavlink/include/mavlink/v1.0 `pkg-config --cflags python-2.7` `pkg-config --cflags SDL_image` `pkg-config --cflags sdl` `pkg-config --cflags libavcodec` `pkg-config --cflags libavformat` `pkg-config --cflags libavutil` `pkg-config --cflags libswscale` -std=c11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/lmmin.o lmmin.c

${OBJECTDIR}/mavlink_control.o: mavlink_control.cpp
	${MKDIR} -p ${OBJECTDIR}
	${RM} "$@.d"
	$(COMPILE.cc) -g `pkg-config --cflags python-2.7` `pkg-config --cflags SDL_image` `pkg-config --cflags sdl` `pkg-config --cflags libavcodec` `pkg-config --cflags libavformat` `pkg-config --cflags libavutil` `pkg-config --cflags libswscale` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/mavlink_control.o mavlink_control.cpp

${OBJECTDIR}/os.o: os.cpp
	${MKDIR} -p ${OBJECTDIR}
	${RM} "$@.d"
	$(COMPILE.cc) -g `pkg-config --cflags python-2.7` `pkg-config --cflags SDL_image` `pkg-config --cflags sdl` `pkg-config --cflags libavcodec` `pkg-config --cflags libavformat` `pkg-config --cflags libavutil` `pkg-config --cflags libswscale` -std=c++11  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/os.o os.cpp

# Subprojects
.build-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r ${CND_BUILDDIR}/${CND_CONF}
	${RM} -r ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/libopencv_videoio.so ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/libopencv_highgui.so ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/libopencv_imgcodecs.so ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/libopencv_imgproc.so ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/libopencv_core.so
	${RM} ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/lanekeeping1213

# Subprojects
.clean-subprojects:

# Enable dependency checking
.dep.inc: .depcheck-impl

include .dep.inc
