################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/learning_parameters/getFilesnames_script.cpp \
../src/learning_parameters/learnninglinearregression.cpp 

OBJS += \
./src/learning_parameters/getFilesnames_script.o \
./src/learning_parameters/learnninglinearregression.o 

CPP_DEPS += \
./src/learning_parameters/getFilesnames_script.d \
./src/learning_parameters/learnninglinearregression.d 


# Each subdirectory must supply rules for building sources it contributes
src/learning_parameters/%.o: ../src/learning_parameters/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


