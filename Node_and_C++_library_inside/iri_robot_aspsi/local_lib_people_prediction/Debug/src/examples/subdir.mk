################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/examples/generate_performance_from_data_example.cpp \
../src/examples/generate_performance_from_data_exampleIROS2019.cpp \
../src/examples/generate_performance_from_data_example_humanoids_side_by_side_2people.cpp \
../src/examples/local_nav_example.cpp \
../src/examples/prediction_example.cpp 

OBJS += \
./src/examples/generate_performance_from_data_example.o \
./src/examples/generate_performance_from_data_exampleIROS2019.o \
./src/examples/generate_performance_from_data_example_humanoids_side_by_side_2people.o \
./src/examples/local_nav_example.o \
./src/examples/prediction_example.o 

CPP_DEPS += \
./src/examples/generate_performance_from_data_example.d \
./src/examples/generate_performance_from_data_exampleIROS2019.d \
./src/examples/generate_performance_from_data_example_humanoids_side_by_side_2people.d \
./src/examples/local_nav_example.d \
./src/examples/prediction_example.d 


# Each subdirectory must supply rules for building sources it contributes
src/examples/%.o: ../src/examples/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


