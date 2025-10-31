################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../local_lib_people_prediction/src/scene_elements/obstacle.cpp \
../local_lib_people_prediction/src/scene_elements/person_abstract.cpp \
../local_lib_people_prediction/src/scene_elements/person_behavior.cpp \
../local_lib_people_prediction/src/scene_elements/person_bhmip.cpp \
../local_lib_people_prediction/src/scene_elements/person_virtual.cpp \
../local_lib_people_prediction/src/scene_elements/robot.cpp 

OBJS += \
./local_lib_people_prediction/src/scene_elements/obstacle.o \
./local_lib_people_prediction/src/scene_elements/person_abstract.o \
./local_lib_people_prediction/src/scene_elements/person_behavior.o \
./local_lib_people_prediction/src/scene_elements/person_bhmip.o \
./local_lib_people_prediction/src/scene_elements/person_virtual.o \
./local_lib_people_prediction/src/scene_elements/robot.o 

CPP_DEPS += \
./local_lib_people_prediction/src/scene_elements/obstacle.d \
./local_lib_people_prediction/src/scene_elements/person_abstract.d \
./local_lib_people_prediction/src/scene_elements/person_behavior.d \
./local_lib_people_prediction/src/scene_elements/person_bhmip.d \
./local_lib_people_prediction/src/scene_elements/person_virtual.d \
./local_lib_people_prediction/src/scene_elements/robot.d 


# Each subdirectory must supply rules for building sources it contributes
local_lib_people_prediction/src/scene_elements/%.o: ../local_lib_people_prediction/src/scene_elements/%.cpp local_lib_people_prediction/src/scene_elements/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


