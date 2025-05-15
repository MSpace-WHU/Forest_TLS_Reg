cmake_minimum_required(VERSION 3.0.2)
project(RegTLSPoints)

set( CMAKE_BUILD_TYPE "Release" )
set(CMAKE_CXX_FLAGS "-std=c++17 -O3")

set(CMAKE_C_FLAGS "-march=native -msse4.2 -mfpmath=sse")
set(CMAKE_CXX_FLAGS "-march=native -msse4.2 -mfpmath=sse")

set(BOOST_ROOT "/usr/local/boost_1_75")
set(BOOST_INCLUDEDIR "${BOOST_ROOT}/include")
set(BOOST_LIBRARYDIR "${BOOST_ROOT}/lib")
set(Boost_USE_STATIC_LIBS OFF) 
find_package(Boost 1.75.0 REQUIRED)

find_package(PCL REQUIRED QUIET)
# set(PCL_DIR "/home/xiaochen/pcl-1.12.0/share/pcl-1.12")
# find_package(PCL 1.12 REQUIRED COMPONENTS)
message(${PCL_INCLUDE_DIRS})
find_package(OpenCV REQUIRED QUIET)
find_package(Eigen3 REQUIRED)
find_package(libLAS REQUIRED)
find_package(GTSAM REQUIRED QUIET)
find_package(libpointmatcher REQUIRED)

find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINATIONS})

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
  ${libpointmatcher_INCLUDE_DIRS}
  ${BOOST_INCLUDEDIR}
)

# trans between two TLS station, from the registrated to the original pose
add_executable(TransToOriPose ./src/TransToOriPos.cpp ./src/Hlp.cpp)
# target_compile_options(TransToOriPose PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(TransToOriPose
  ${PCL_LIBRARIES} 
  ${OpenCV_LIBRARIES}
  ${libLAS_LIBRARIES}
)

# trans between two TLS station, from the registrated to the original pose
add_executable(TransToCommon ./src/TransToCommon.cpp ./src/Hlp.cpp)
# target_compile_options(TransToOriPose PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(TransToCommon
  ${PCL_LIBRARIES} 
  ${OpenCV_LIBRARIES}
  ${libLAS_LIBRARIES}
)

# trans between two TLS station, from the registrated to the original pose
add_executable(TransToCommonByOripose ./src/TransToCommonByOripose.cpp ./src/Hlp.cpp)
# target_compile_options(TransToOriPose PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(TransToCommonByOripose
  ${PCL_LIBRARIES} 
  ${OpenCV_LIBRARIES}
  ${libLAS_LIBRARIES}
)

# Reg between two TLS station
add_executable(RegTLSPoints ./src/RegTLS.cpp ./src/Hlp.cpp ./src/GenAlg.cpp ./src/Point2img.cpp ./src/FEC.cpp ./src/TriObj.cpp)
target_compile_options(RegTLSPoints PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(RegTLSPoints
  ${PCL_LIBRARIES} 
  ${OpenCV_LIBRARIES}
  ${OpenMP_CXX_FLAGS}
  ${libLAS_LIBRARIES}
  # ${libpointmatcher_LIBRARIES}
  ${Boost_LIBRARIES}
  # # libfast_gicp.so
  libsmall_gicp.so
  libCSF.a
)

# Chain Reg TLS
add_executable(ChainRegTLSPoints ./src/ChainRegTLS.cpp ./src/Hlp.cpp ./src/GenAlg.cpp ./src/Point2img.cpp ./src/FEC.cpp ./src/TriObj.cpp)
target_compile_options(ChainRegTLSPoints PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(ChainRegTLSPoints
  ${PCL_LIBRARIES} 
  ${OpenCV_LIBRARIES}
  ${OpenMP_CXX_FLAGS}
  ${libLAS_LIBRARIES}
  # ${libpointmatcher_LIBRARIES}
  ${Boost_LIBRARIES}
  # libfast_gicp.so
  libsmall_gicp.so
  libCSF.a
)

# Multi Reg TLS
add_executable(MultiRegTLSPoints ./src/MultiRegTLS.cpp ./src/Hlp.cpp ./src/gtsamOpti.cpp ./src/GenAlg.cpp ./src/Point2img.cpp ./src/FEC.cpp ./src/TriObj.cpp)
target_compile_options(MultiRegTLSPoints PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(MultiRegTLSPoints
  ${PCL_LIBRARIES} 
  ${OpenCV_LIBRARIES}
  ${OpenMP_CXX_FLAGS}
  ${libLAS_LIBRARIES}
  # ${libpointmatcher_LIBRARIES}
  ${Boost_LIBRARIES}
  # libfast_gicp.so
  libsmall_gicp.so
  libCSF.a
  gtsam
)

# Reg the Tongji tree dataset
add_executable(MultiRegTongji ./src/MultiRegTongji.cpp ./src/Hlp.cpp ./src/gtsamOpti.cpp ./src/GenAlg.cpp ./src/Point2img.cpp ./src/FEC.cpp ./src/TriObj.cpp)
target_compile_options(MultiRegTongji PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(MultiRegTongji
  ${PCL_LIBRARIES} 
  ${OpenCV_LIBRARIES}
  ${OpenMP_CXX_FLAGS}
  ${libLAS_LIBRARIES}
  # ${libpointmatcher_LIBRARIES}
  ${Boost_LIBRARIES}
  # libfast_gicp.so
  libsmall_gicp.so
  libCSF.a
  gtsam
)

# Multi Reg FGI
add_executable(MultiRegFGI ./src/MultiRegFGI.cpp ./src/Hlp.cpp ./src/gtsamOpti.cpp ./src/GenAlg.cpp ./src/Point2img.cpp ./src/FEC.cpp ./src/TriObj.cpp)
target_compile_options(MultiRegFGI PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(MultiRegFGI
  ${PCL_LIBRARIES} 
  ${OpenCV_LIBRARIES}
  ${OpenMP_CXX_FLAGS}
  ${libLAS_LIBRARIES}
  # ${libpointmatcher_LIBRARIES}
  ${Boost_LIBRARIES}
  # libfast_gicp.so
  libsmall_gicp.so
  libCSF.a
  gtsam
)

# Reg the ETH
add_executable(MultiRegETH ./src/MultiRegETH.cpp ./src/Hlp.cpp ./src/gtsamOpti.cpp ./src/GenAlg.cpp ./src/Point2img.cpp ./src/FEC.cpp ./src/TriObj.cpp)
target_compile_options(MultiRegETH PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(MultiRegETH
  ${PCL_LIBRARIES} 
  ${OpenCV_LIBRARIES}
  ${OpenMP_CXX_FLAGS}
  ${libLAS_LIBRARIES}
  # ${libpointmatcher_LIBRARIES}
  ${Boost_LIBRARIES}
  # libfast_gicp.so
  libsmall_gicp.so
  libCSF.a
  gtsam
)

# trans the pcd to las file
add_executable(pcdToLas ./src/pcdToLas.cpp ./src/Hlp.cpp)
target_compile_options(pcdToLas PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(pcdToLas
  ${PCL_LIBRARIES} 
  ${libLAS_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

# test the dfs algorithm
add_executable(dfsTest ./src/dfsTest.cpp)

# Reg between two tower station
add_executable(TowerReg ./src/TowerReg.cpp ./src/Hlp.cpp ./src/GenAlg.cpp ./src/Point2img.cpp ./src/FEC.cpp ./src/TriObj.cpp)
target_compile_options(TowerReg PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(TowerReg
  ${PCL_LIBRARIES} 
  ${OpenCV_LIBRARIES}
  ${OpenMP_CXX_FLAGS}
  ${libLAS_LIBRARIES}
  # ${libpointmatcher_LIBRARIES}
  ${Boost_LIBRARIES}
  # libfast_gicp.so
  libsmall_gicp.so
  libCSF.a
)

# Reg Multi tower stations
add_executable(MultiTowerReg ./src/MultiTowerReg.cpp ./src/Hlp.cpp ./src/GenAlg.cpp ./src/Point2img.cpp ./src/FEC.cpp ./src/TriObj.cpp)
target_compile_options(MultiTowerReg PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(MultiTowerReg
  ${PCL_LIBRARIES} 
  ${OpenCV_LIBRARIES}
  ${OpenMP_CXX_FLAGS}
  ${libLAS_LIBRARIES}
  # ${libpointmatcher_LIBRARIES}
  ${Boost_LIBRARIES}
  # libfast_gicp.so
  libsmall_gicp.so
  libCSF.a
)

# Chain Reg tower stations
add_executable(ChainTowerReg ./src/ChainTowerReg.cpp ./src/Hlp.cpp ./src/GenAlg.cpp ./src/Point2img.cpp ./src/FEC.cpp ./src/TriObj.cpp)
target_compile_options(ChainTowerReg PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(ChainTowerReg
  ${PCL_LIBRARIES} 
  ${OpenCV_LIBRARIES}
  ${OpenMP_CXX_FLAGS}
  ${libLAS_LIBRARIES}
  # ${libpointmatcher_LIBRARIES}
  ${Boost_LIBRARIES}
  # libfast_gicp.so
  libsmall_gicp.so
  libCSF.a
)
