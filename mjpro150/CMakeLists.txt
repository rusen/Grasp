cmake_minimum_required(VERSION 2.4.6)
if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

message("Starteda")

# Add compile options here.
if (APPLE)
	add_compile_options(-O0 -g3 -I/.include)
elseif (UNIX)
	add_compile_options(-O0 -g3 -I/.include -mavx)
endif ()

#set(OpenCV_DIR "/home/xxa344/Desktop/opencv-3.2.0/release")

set(CMAKE_BUILD_TYPE Release)
set (CMAKE_CXX_STANDARD 11)
set(OpenCV_LIBS opencv_core opencv_imgproc opencv_calib3d opencv_video opencv_features2d opencv_ml opencv_highgui opencv_objdetect opencv_contrib opencv_legacy opencv_gpu)
set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)

if (APPLE)
	add_subdirectory(./lib/macos)
elseif (UNIX)
	add_subdirectory(./lib/unix)
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
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

if (NOT APPLE)
	find_package(OpenMP)
	if(OPENMP_FOUND)
		message("Found OpenMP")
		include_directories(${OpenMP_INCLUDE_DIRS})
		set(LIBS ${LIBS} ${OpenMP_LIBRARIES})
		set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -DHAVE_OMP")
	endif(OPENMP_FOUND)
endif()

find_package(PCL 1.3 QUIET REQUIRED COMPONENTS io)
if(PCL_FOUND)
  message("Found PCL")
  include_directories(${PCL_INCLUDE_DIRS})
  # in case your PCL installation is part of ros you might need to add this include path
  #include_directories(${PCL_INCLUDE_DIRS} /opt/ros/<ros_distro>/include/)
  set(LIBS ${LIBS} ${PCL_LIBRARIES})
  add_definitions(${PCL_DEFINITIONS})
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -DHAVE_PCL")
endif(PCL_FOUND)

find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui calib3d video features2d ml objdetect)
message("Found OpenCV")
include_directories(${OpenCV_INCLUDE_DIRS})
set(LIBS ${LIBS} ${OpenCV_LIBS})

add_library(${PROJECT_NAME} src/sensor/kinectSimulator.cpp src/sensor/noiseutils.cpp)

if (APPLE)
	target_link_libraries(${PROJECT_NAME} -lmujoco150 -L./lib/macos -lglfw.3 assimp CGAL noise ${OpenCV_LIBS} ${LIBS} ${OPENGL_gl_LIBRARY})
elseif (UNIX)
    target_link_libraries(${PROJECT_NAME} -lmujoco150 -L./lib/unix -lGL -lglew -lglfw assimp CGAL noise ${OpenCV_LIBS} ${LIBS} ${OPENGL_gl_LIBRARY})
endif ()

file(GLOB_RECURSE CTRL_SOURCES "src/controller/*.cpp")
file(GLOB_RECURSE PLAN_SOURCES "src/planner/*.cpp")
file(GLOB_RECURSE SNSR_SOURCES "src/sensor/*.cpp")

if (UNIX)
add_executable(basicGrasp src/basicGrasp.cpp ${CTRL_SOURCES} ${PLAN_SOURCES} ${SNSR_SOURCES})
endif (UNIX)

add_executable(basicGrasp src/basicGrasp.cpp ${CTRL_SOURCES} ${PLAN_SOURCES} ${SNSR_SOURCES})

target_link_libraries(basicGrasp ${PROJECT_NAME})
target_link_libraries(basicGrasp ${OpenCV_LIBS})
