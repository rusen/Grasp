cmake_minimum_required(VERSION 2.4.6)
if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

message("Starteda")

# Add compile options here.
if (APPLE)
	add_compile_options(-O3 -I${PROJECT_SOURCE_DIR}/include -frounding-math)
elseif (UNIX)
	add_compile_options(-O3 -std=c++11 -fopenmp -DBOOST_NO_CXX11_SCOPED_ENUMS -I${PROJECT_SOURCE_DIR}/include -mavx -frounding-math)
endif ()

set(CMAKE_BUILD_TYPE Debug)
set (CMAKE_CXX_STANDARD 11)
#set(OpenCV_LIBS opencv_core opencv_imgproc opencv_calib3d opencv_video opencv_features2d opencv_ml opencv_highgui opencv_objdetect opencv_contrib opencv_legacy opencv_gpu)
set(OpenCV_LIBS opencv_core opencv_imgproc opencv_highgui)
set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)

if (APPLE)
	add_subdirectory(${PROJECT_SOURCE_DIR}/lib/macos)
elseif (UNIX)
	add_subdirectory(${PROJECT_SOURCE_DIR}/lib/unix)
endif ()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
#set the path for project includes
include_directories(${PROJECT_SOURCE_DIR}/include)

# There exist different versions of the assimp library for different 
# ubuntu distros. 
execute_process(COMMAND lsb_release -sc 
  OUTPUT_VARIABLE _distro OUTPUT_STRIP_TRAILING_WHITESPACE)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DHAVE_${_distro}")
message("Compiling for Ubuntu version ${_distro}")

find_package(OpenGL REQUIRED)

find_package(PCL 1.7 REQUIRED COMPONENTS common features io search)
if(PCL_FOUND)
  message("Found PCL")
  include_directories(${PCL_INCLUDE_DIRS})
  link_directories(${PCL_LIBRARY_DIRS})
  add_definitions(${PCL_DEFINITIONS})
  #  set(LIBS ${LIBS} ${PCL_LIBRARIES})
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -DHAVE_PCL")
endif(PCL_FOUND)

find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui)
message("Found OpenCV")
include_directories(${OpenCV_INCLUDE_DIRS})
set(LIBS ${LIBS} ${OpenCV_LIBS})

find_library(NOISE_LIBRARY noise /usr/local)

add_library(${PROJECT_NAME} src/sensor/kinectSimulator.cpp src/sensor/noiseutils.cpp)

if (APPLE)
	target_link_libraries(${PROJECT_NAME} -lmujoco150 -L${PROJECT_SOURCE_DIR}/lib/macos curl curlpp ${PCL_LIBRARIES} ${NOISE_LIBRARY} -lglfw.3 ${OpenCV_LIBS} ${LIBS} ${OPENGL_gl_LIBRARY})
elseif (UNIX)
	target_link_libraries(${PROJECT_NAME}  -fopenmp -lmujoco150 -DBOOST_NO_CXX11_SCOPED_ENUMS ${PCL_LIBRARIES} -lboost_system -L${PROJECT_SOURCE_DIR}/lib/unix ${PROJECT_SOURCE_DIR}/lib/unix/libglfw.so.3 curl curlpp -lGL -lglew noise ${OpenCV_LIBS} ${LIBS} ${OPENGL_gl_LIBRARY})
endif ()

file(GLOB_RECURSE EXTRA_SOURCES "src/*/*.cpp")

add_executable(basicGrasp src/basicGrasp.cpp ${EXTRA_SOURCES})
add_executable(playlog src/playlog.cpp ${EXTRA_SOURCES})
add_executable(uploadFiles src/uploadFiles.cpp ${EXTRA_SOURCES})

target_link_libraries(basicGrasp ${PROJECT_NAME})
target_link_libraries(basicGrasp ${OpenCV_LIBS})
target_link_libraries(playlog ${PROJECT_NAME})
target_link_libraries(playlog ${OpenCV_LIBS})
target_link_libraries(uploadFiles ${PROJECT_NAME})
