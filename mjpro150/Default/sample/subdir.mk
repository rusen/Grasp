################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../sample/basic.cpp \
../sample/compile.cpp \
../sample/derivative.cpp \
../sample/grasp.cpp \
../sample/graspSimulate.cpp \
../sample/record.cpp \
../sample/simulate.cpp \
../sample/test.cpp 

OBJS += \
./sample/basic.o \
./sample/compile.o \
./sample/derivative.o \
./sample/grasp.o \
./sample/graspSimulate.o \
./sample/record.o \
./sample/simulate.o \
./sample/test.o 

CPP_DEPS += \
./sample/basic.d \
./sample/compile.d \
./sample/derivative.d \
./sample/grasp.d \
./sample/graspSimulate.d \
./sample/record.d \
./sample/simulate.d \
./sample/test.d 


# Each subdirectory must supply rules for building sources it contributes
sample/%.o: ../sample/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I"/Users/rusi/Workspace/Grasp/mjpro150/include" -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


