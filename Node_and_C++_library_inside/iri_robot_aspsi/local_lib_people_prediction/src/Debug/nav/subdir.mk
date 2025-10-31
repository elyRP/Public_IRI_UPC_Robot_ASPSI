################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../nav/force_reactive_robot_companion.cpp \
../nav/force_reactive_robot_companion_learning.cpp \
../nav/plan_local_nav.cpp \
../nav/plan_local_nav_old_ok.cpp \
../nav/plan_local_nav_person_companion.cpp 

OBJS += \
./nav/force_reactive_robot_companion.o \
./nav/force_reactive_robot_companion_learning.o \
./nav/plan_local_nav.o \
./nav/plan_local_nav_old_ok.o \
./nav/plan_local_nav_person_companion.o 

CPP_DEPS += \
./nav/force_reactive_robot_companion.d \
./nav/force_reactive_robot_companion_learning.d \
./nav/plan_local_nav.d \
./nav/plan_local_nav_old_ok.d \
./nav/plan_local_nav_person_companion.d 


# Each subdirectory must supply rules for building sources it contributes
nav/%.o: ../nav/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


