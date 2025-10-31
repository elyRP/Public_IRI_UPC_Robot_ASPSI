################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/scene_elements/obstacle.cpp \
../src/scene_elements/person_abstract.cpp \
../src/scene_elements/person_behavior.cpp \
../src/scene_elements/person_bhmip.cpp \
../src/scene_elements/person_virtual.cpp \
../src/scene_elements/robot.cpp 

OBJS += \
./src/scene_elements/obstacle.o \
./src/scene_elements/person_abstract.o \
./src/scene_elements/person_behavior.o \
./src/scene_elements/person_bhmip.o \
./src/scene_elements/person_virtual.o \
./src/scene_elements/robot.o 

CPP_DEPS += \
./src/scene_elements/obstacle.d \
./src/scene_elements/person_abstract.d \
./src/scene_elements/person_behavior.d \
./src/scene_elements/person_bhmip.d \
./src/scene_elements/person_virtual.d \
./src/scene_elements/robot.d 


# Each subdirectory must supply rules for building sources it contributes
src/scene_elements/%.o: ../src/scene_elements/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


