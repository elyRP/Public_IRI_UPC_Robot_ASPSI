################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../local_lib_people_prediction/src/test/test_geometry.cpp \
../local_lib_people_prediction/src/test/test_person.cpp \
../local_lib_people_prediction/src/test/test_scene.cpp 

OBJS += \
./local_lib_people_prediction/src/test/test_geometry.o \
./local_lib_people_prediction/src/test/test_person.o \
./local_lib_people_prediction/src/test/test_scene.o 

CPP_DEPS += \
./local_lib_people_prediction/src/test/test_geometry.d \
./local_lib_people_prediction/src/test/test_person.d \
./local_lib_people_prediction/src/test/test_scene.d 


# Each subdirectory must supply rules for building sources it contributes
local_lib_people_prediction/src/test/%.o: ../local_lib_people_prediction/src/test/%.cpp local_lib_people_prediction/src/test/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


