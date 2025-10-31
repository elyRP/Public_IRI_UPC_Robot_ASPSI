################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../local_lib_people_prediction/build/CMakeFiles/3.10.2/CompilerIdC/CMakeCCompilerId.c 

OBJS += \
./local_lib_people_prediction/build/CMakeFiles/3.10.2/CompilerIdC/CMakeCCompilerId.o 

C_DEPS += \
./local_lib_people_prediction/build/CMakeFiles/3.10.2/CompilerIdC/CMakeCCompilerId.d 


# Each subdirectory must supply rules for building sources it contributes
local_lib_people_prediction/build/CMakeFiles/3.10.2/CompilerIdC/%.o: ../local_lib_people_prediction/build/CMakeFiles/3.10.2/CompilerIdC/%.c local_lib_people_prediction/build/CMakeFiles/3.10.2/CompilerIdC/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


