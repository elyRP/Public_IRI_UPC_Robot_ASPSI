################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../examples/generate_performance_from_data_example.cpp \
../examples/generate_performance_from_data_exampleIROS2019.cpp \
../examples/generate_performance_from_data_example_humanoids_side_by_side_2people.cpp \
../examples/local_nav_example.cpp \
../examples/prediction_example.cpp 

OBJS += \
./examples/generate_performance_from_data_example.o \
./examples/generate_performance_from_data_exampleIROS2019.o \
./examples/generate_performance_from_data_example_humanoids_side_by_side_2people.o \
./examples/local_nav_example.o \
./examples/prediction_example.o 

CPP_DEPS += \
./examples/generate_performance_from_data_example.d \
./examples/generate_performance_from_data_exampleIROS2019.d \
./examples/generate_performance_from_data_example_humanoids_side_by_side_2people.d \
./examples/local_nav_example.d \
./examples/prediction_example.d 


# Each subdirectory must supply rules for building sources it contributes
examples/%.o: ../examples/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


