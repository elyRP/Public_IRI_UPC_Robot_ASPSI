################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../local_lib_people_prediction/src/nav/force_reactive_robot_companion.cpp \
../local_lib_people_prediction/src/nav/force_reactive_robot_companion_learning.cpp \
../local_lib_people_prediction/src/nav/plan_local_nav.cpp \
../local_lib_people_prediction/src/nav/plan_local_nav_person_companion.cpp 

OBJS += \
./local_lib_people_prediction/src/nav/force_reactive_robot_companion.o \
./local_lib_people_prediction/src/nav/force_reactive_robot_companion_learning.o \
./local_lib_people_prediction/src/nav/plan_local_nav.o \
./local_lib_people_prediction/src/nav/plan_local_nav_person_companion.o 

CPP_DEPS += \
./local_lib_people_prediction/src/nav/force_reactive_robot_companion.d \
./local_lib_people_prediction/src/nav/force_reactive_robot_companion_learning.d \
./local_lib_people_prediction/src/nav/plan_local_nav.d \
./local_lib_people_prediction/src/nav/plan_local_nav_person_companion.d 


# Each subdirectory must supply rules for building sources it contributes
local_lib_people_prediction/src/nav/%.o: ../local_lib_people_prediction/src/nav/%.cpp local_lib_people_prediction/src/nav/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


