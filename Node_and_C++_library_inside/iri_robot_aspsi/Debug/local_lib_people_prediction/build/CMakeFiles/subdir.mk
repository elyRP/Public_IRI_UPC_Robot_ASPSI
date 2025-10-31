################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CXX_SRCS += \
../local_lib_people_prediction/build/CMakeFiles/feature_tests.cxx 

C_SRCS += \
../local_lib_people_prediction/build/CMakeFiles/feature_tests.c 

CXX_DEPS += \
./local_lib_people_prediction/build/CMakeFiles/feature_tests.d 

OBJS += \
./local_lib_people_prediction/build/CMakeFiles/feature_tests.o 

C_DEPS += \
./local_lib_people_prediction/build/CMakeFiles/feature_tests.d 


# Each subdirectory must supply rules for building sources it contributes
local_lib_people_prediction/build/CMakeFiles/%.o: ../local_lib_people_prediction/build/CMakeFiles/%.c local_lib_people_prediction/build/CMakeFiles/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

local_lib_people_prediction/build/CMakeFiles/%.o: ../local_lib_people_prediction/build/CMakeFiles/%.cxx local_lib_people_prediction/build/CMakeFiles/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


