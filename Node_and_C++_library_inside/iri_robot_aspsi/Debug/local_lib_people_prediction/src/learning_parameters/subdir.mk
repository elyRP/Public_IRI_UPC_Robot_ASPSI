################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../local_lib_people_prediction/src/learning_parameters/getFilesnames_script.cpp \
../local_lib_people_prediction/src/learning_parameters/learnninglinearregression.cpp 

OBJS += \
./local_lib_people_prediction/src/learning_parameters/getFilesnames_script.o \
./local_lib_people_prediction/src/learning_parameters/learnninglinearregression.o 

CPP_DEPS += \
./local_lib_people_prediction/src/learning_parameters/getFilesnames_script.d \
./local_lib_people_prediction/src/learning_parameters/learnninglinearregression.d 


# Each subdirectory must supply rules for building sources it contributes
local_lib_people_prediction/src/learning_parameters/%.o: ../local_lib_people_prediction/src/learning_parameters/%.cpp local_lib_people_prediction/src/learning_parameters/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


