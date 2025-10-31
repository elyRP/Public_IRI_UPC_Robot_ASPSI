################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../test/test_geometry.cpp \
../test/test_person.cpp \
../test/test_scene.cpp 

OBJS += \
./test/test_geometry.o \
./test/test_person.o \
./test/test_scene.o 

CPP_DEPS += \
./test/test_geometry.d \
./test/test_person.d \
./test/test_scene.d 


# Each subdirectory must supply rules for building sources it contributes
test/%.o: ../test/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


