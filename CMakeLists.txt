PROJECT(opencv_calibrate)
cmake_minimum_required(VERSION 3.5)
set (EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)


include_directories (source/camera/camera_uvc)
include_directories (source/camera)
include_directories (source/3rd_interface/ffmpeg)
include_directories (source/3rd_interface/opencv)
include_directories (source/framebuffer)
aux_source_directory(source/camera/camera_uvc    BASE1_SRC)  
aux_source_directory(source/framebuffer          BASE2_SRC)
aux_source_directory(source/3rd_interface/opencv BASE3_SRC)
aux_source_directory(source/3rd_interface/ffmpeg BASE4_SRC)

option(HI3516 "USE HI3516 LIB"  OFF)
option(X86    "USE X86 LIB" ON)

if(HI3516)
    #添加环境所需的头文件
    MESSAGE(USE_HI3516_LIB)
        #设置两个库所在位置
        set(FFMPEGDIR /home/swann/SDK/HI3516/SDK/ffmpeg)
        set(OPENCVDIR /home/swann/SDK/HI3516/SDK/opencv/INSTALL)
        list(APPEND FFmpeg_LIBS  avformat avdevice avcodec avutil avfilter postproc swscale swresample)
        add_link_options(-L${FFMPEGDIR}/lib -lpthread)
        list(APPEND OpenCV_LIBS      opencv_imgproc opencv_core opencv_highgui opencv_video)
        add_link_options(-L${OPENCVDIR}/lib)
    
    
        include_directories (${FFMPEGDIR}/include)
        include_directories (${OPENCVDIR}/include)
    
        #添加平台摄像头文件夹
        aux_source_directory(source/camera/camera_gc2053   PLATFORM_CAMERA)  
        include_directories (source/camera/camera_gc2053)
    
    
        set (MPPDIR /home/swann/SDK/HI3516/MYMMP/mpp)
        set (COMMON_DIR   ${MPPDIR}/sample/common)
        set (HISILIB      ${MPPDIR}/lib)
        include_directories (${MPPDIR}/include)
        include_directories (${MPPDIR}/sample/audio/adp)
        include_directories (${MPPDIR}/sample/common)
        include_directories (${MPPDIR}/sample/vio)
        
        list(APPEND HISI_LIBS  isp mpi hdmi sns_gc2053 dnvqe VoiceEngine upvqe securec securec
        sns_imx327 sns_imx327_2l sns_imx307 sns_imx307_2l sns_imx458 sns_mn34220
        sns_os05a sns_os08a10 sns_sc4210 sns_ov12870 sns_os04b10 sns_imx415 
        sns_imx219 sns_imx274 sns_imx335 _hiae _hidehaze _hildci _hiawb _hidrc _hicalcflicker)
        add_link_options(-L${HISILIB})
    
    
        add_compile_options(
        -std=c++11 -lpthread -lm -ldl -DISP_V2 -lstdc++ -Wall -mcpu=cortex-a7 -mfloat-abi=softfp 
        -mfpu=neon-vfpv4 -fno-aggressive-loop-optimizations -ldl -ffunction-sections -fdata-sections 
        -O2 -fstack-protector-strong -fPIC -Wall  -DHI_RELEASE -Wno-error=implicit-function-declaration 
        -DVER_X=1 -DVER_Y=0 -DVER_Z=0 -DVER_P=0 -DVER_B=10 -DUSER_BIT_32 -DKERNEL_BIT_32 -Wno-date-time	
        -DSENSOR0_TYPE=GALAXYCORE_GC2053_MIPI_2M_30FPS_10BIT 
        -DSENSOR1_TYPE=GALAXYCORE_GC2053_MIPI_2M_30FPS_10BIT	
        -DHI_ACODEC_TYPE_INNER -DHI_ACODEC_TYPE_HDMI ) 
        set(CMAKE_C_COMPILER "arm-himix200-linux-gcc")
        set(CMAKE_CXX_COMPILER "arm-himix200-linux-g++")


        #UVC图像捕捉程序
        add_executable(opencv_cap_picture    ${PLATFORM_CAMERA}   ${BASE1_SRC} ${BASE2_SRC} ${BASE3_SRC} ${BASE4_SRC} source/opencv_cap_picture.cpp)
        target_link_libraries(opencv_cap_picture ${OpenCV_LIBS})
        target_link_libraries(opencv_cap_picture ${FFmpeg_LIBS})
        target_link_libraries(opencv_cap_picture ${HISI_LIBS})

elseif(X86)
    MESSAGE(USE_X86_LIB)
    add_compile_options(-std=c++11)
    set(FFMPEGDIR /home/swann/SDK/X86/SDK/ffmpeg)
    set(FFMPEGDIR /home/swann/SDK/X86/SDK/opencv/install)
    list(APPEND FFmpeg_LIBS  avformat avdevice avcodec avutil avfilter postproc swscale swresample)
    add_link_options(-L${FFMPEGDIR}/lib -lpthread)
    # find_package(OpenCV  REQUIRED)
    # MESSAGE(${OpenCV_INCLUDE_DIRS})
    # include_directories(${OpenCV_INCLUDE_DIRS}) 
    list(APPEND OpenCV_LIBS opencv_world)
    add_link_options(-L${OPENCVDIR}/lib)
    include_directories (${FFMPEGDIR}/include)
    include_directories (${OPENCVDIR}/include)

    #图像点分析程序
    add_executable(opencv_point_det             source/opencv_point_det.cpp)
    #计算内参和畸变参数程序
    add_executable(opencv_calibrate_inner       source/opencv_calibrate_inner.cpp)
    #通过内参和数据点计算外参程序
    add_executable(opencv_calibrate_outer       source/opencv_calibrate_outer.cpp)
    # #相机消除畸变的程序
    # add_executable(opencv_remap               source/opencv_remap.cpp)
    # directory of opencv library
    link_directories(${OpenCV_LIBRARY_DIRS})
    # opencv libraries
    # target_link_libraries(opencv_cap_picture ${OpenCV_LIBS})
    target_link_libraries(opencv_calibrate_inner ${OpenCV_LIBS})
    target_link_libraries(opencv_point_det ${OpenCV_LIBS})
    target_link_libraries(opencv_calibrate_outer ${OpenCV_LIBS})
    # target_link_libraries(opencv_remap ${OpenCV_LIBS})

endif()


