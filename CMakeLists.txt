cmake_minimum_required(VERSION 3.0.2)
project(ForestTLSReg)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find PCL, version 1.8 (system), may confilict with GTSAM or smallGICP 
# find_package(PCL 1.8 REQUIRED QUIET)

# find PCL, self builded, for smallGICP
set(PCL_DIR "/home/xiaochen/pcl-1.12.0/share/pcl-1.12")
find_package(PCL 1.12 REQUIRED)
# message(${PCL_INCLUDE_DIRS})
# message(STATUS "PCL_LIBRARIES: ${PCL_LIBRARIES}")

# find third party library
find_package(OpenCV REQUIRED QUIET)
find_package(Eigen3 REQUIRED)
find_package(libLAS REQUIRED)
find_package(GTSAM REQUIRED QUIET)
find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

# include directories
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
)

# link directories
link_directories(
	include
	${PCL_LIBRARY_DIRS}
  ${OpenCV_LIBRARY_DIRS}
)

# set the global path variable
set(PROJECT_ROOT ${CMAKE_SOURCE_DIR})
add_compile_definitions(PROJECT_PATH="${PROJECT_ROOT}")

# find file's global path
file(GLOB utils_sources ./src/utils/*.cpp)
file(GLOB gtsam_opti_sources ./src/gtsam_opti/*.cpp)

##########################################################
#####################Useful Function######################
##########################################################
# trans the pcd to las file
add_executable(pcdToLas ./src/function/pcdToLas.cpp ./src/utils/Hlp.cpp)
target_compile_options(pcdToLas PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(pcdToLas
  ${PCL_LIBRARIES} 
  ${libLAS_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

# trans between two TLS station, from the registrated to the original pose
add_executable(TransToOriPose ./src/function/TransToOriPos.cpp ./src/utils/Hlp.cpp)
target_link_libraries(TransToOriPose
  ${PCL_LIBRARIES} 
  ${OpenCV_LIBRARIES}
  ${libLAS_LIBRARIES}
)

# trans between two TLS station, from the registrated to the original pose
add_executable(TransToCommon ./src/function/TransToCommon.cpp ./src/utils/Hlp.cpp)
target_link_libraries(TransToCommon
  ${PCL_LIBRARIES} 
  ${OpenCV_LIBRARIES}
  ${libLAS_LIBRARIES}
)

# trans between two TLS station, from the registrated to the original pose
add_executable(TransToCommonByOripose ./src/function/TransToCommonByOripose.cpp ./src/utils/Hlp.cpp)
target_link_libraries(TransToCommonByOripose
  ${PCL_LIBRARIES} 
  ${OpenCV_LIBRARIES}
  ${libLAS_LIBRARIES}
)
##########################################################
#####################Useful Function######################
##########################################################


##########################################################
########################Demo Node#########################
##########################################################
# Reg between two TLS stations
add_executable(Reg2TLSPoints ./src/demo/Reg2TLS.cpp ${utils_sources})
target_compile_options(Reg2TLSPoints PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(Reg2TLSPoints
  ${PCL_LIBRARIES} 
  ${OpenCV_LIBRARIES}
  ${OpenMP_CXX_FLAGS}
  ${libLAS_LIBRARIES}
  libCSF.a
  libsmall_gicp.so
)

# Reg TLS Chain
add_executable(ChainRegTLSPoints ./src/demo/ChainRegTLS.cpp ${utils_sources})
target_compile_options(ChainRegTLSPoints PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(ChainRegTLSPoints
  ${PCL_LIBRARIES} 
  ${OpenCV_LIBRARIES}
  ${OpenMP_CXX_FLAGS}
  ${libLAS_LIBRARIES}
  libsmall_gicp.so
  libCSF.a
)

# Multi Reg TLS
add_executable(MultiRegTLSPoints ./src/demo/MultiRegTLS.cpp ${utils_sources} ${gtsam_opti_sources})
target_compile_options(MultiRegTLSPoints PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(MultiRegTLSPoints
  ${PCL_LIBRARIES} 
  ${OpenCV_LIBRARIES}
  ${OpenMP_CXX_FLAGS}
  ${libLAS_LIBRARIES}
  libsmall_gicp.so
  libCSF.a
  gtsam
)

# Reg Tongji tree dataset
add_executable(MultiRegTongji ./src/demo/MultiRegTongji.cpp ${utils_sources} ${gtsam_opti_sources})
target_compile_options(MultiRegTongji PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(MultiRegTongji
  ${PCL_LIBRARIES} 
  ${OpenCV_LIBRARIES}
  ${OpenMP_CXX_FLAGS}
  ${libLAS_LIBRARIES}
  libsmall_gicp.so
  libCSF.a
  gtsam
)

# Multi Reg FGI
add_executable(MultiRegFGI ./src/demo/MultiRegFGI.cpp ${utils_sources} ${gtsam_opti_sources})
target_compile_options(MultiRegFGI PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(MultiRegFGI
  ${PCL_LIBRARIES} 
  ${OpenCV_LIBRARIES}
  ${OpenMP_CXX_FLAGS}
  ${libLAS_LIBRARIES}
  libsmall_gicp.so
  libCSF.a
  gtsam
)

# Reg the ETH
add_executable(MultiRegETH ./src/demo/MultiRegETH.cpp ${utils_sources} ${gtsam_opti_sources})
target_compile_options(MultiRegETH PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(MultiRegETH
  ${PCL_LIBRARIES} 
  ${OpenCV_LIBRARIES}
  ${OpenMP_CXX_FLAGS}
  ${libLAS_LIBRARIES}
  libsmall_gicp.so
  libCSF.a
  gtsam
)
##########################################################
########################Demo Node#########################
##########################################################


##########################################################
########################Test Node#########################
##########################################################

# Reg between two tower station
add_executable(TowerReg ./src/test/TowerReg.cpp ${utils_sources})
target_compile_options(TowerReg PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(TowerReg
  ${PCL_LIBRARIES} 
  ${OpenCV_LIBRARIES}
  ${OpenMP_CXX_FLAGS}
  ${libLAS_LIBRARIES}
  libsmall_gicp.so
  libCSF.a
)

# Reg Multi tower stations
add_executable(MultiTowerReg ./src/test/MultiTowerReg.cpp ${utils_sources})
target_compile_options(MultiTowerReg PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(MultiTowerReg
  ${PCL_LIBRARIES} 
  ${OpenCV_LIBRARIES}
  ${OpenMP_CXX_FLAGS}
  ${libLAS_LIBRARIES}
  libsmall_gicp.so
  libCSF.a
)

# Chain Reg tower stations
add_executable(ChainTowerReg ./src/test/ChainTowerReg.cpp ${utils_sources})
target_compile_options(ChainTowerReg PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(ChainTowerReg
  ${PCL_LIBRARIES} 
  ${OpenCV_LIBRARIES}
  ${OpenMP_CXX_FLAGS}
  ${libLAS_LIBRARIES}
  libsmall_gicp.so
  libCSF.a
)

# test the dfs algorithm
add_executable(dfsTest ./src/test/dfsTest.cpp)

##########################################################
########################Test Node#########################
##########################################################