################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../local_lib_people_prediction/build/CMakeFiles/3.10.2/CompilerIdCXX/CMakeCXXCompilerId.cpp 

OBJS += \
./local_lib_people_prediction/build/CMakeFiles/3.10.2/CompilerIdCXX/CMakeCXXCompilerId.o 

CPP_DEPS += \
./local_lib_people_prediction/build/CMakeFiles/3.10.2/CompilerIdCXX/CMakeCXXCompilerId.d 


# Each subdirectory must supply rules for building sources it contributes
local_lib_people_prediction/build/CMakeFiles/3.10.2/CompilerIdCXX/%.o: ../local_lib_people_prediction/build/CMakeFiles/3.10.2/CompilerIdCXX/%.cpp local_lib_people_prediction/build/CMakeFiles/3.10.2/CompilerIdCXX/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


