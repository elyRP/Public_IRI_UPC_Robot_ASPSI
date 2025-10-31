################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/akp_local_planner_alg.cpp \
../src/akp_local_planner_alg_indigo_hydro_kinetic.cpp \
../src/akp_local_planner_alg_node.cpp \
../src/akp_local_planner_alg_node_indigo_hydro_kinetic.cpp 

OBJS += \
./src/akp_local_planner_alg.o \
./src/akp_local_planner_alg_indigo_hydro_kinetic.o \
./src/akp_local_planner_alg_node.o \
./src/akp_local_planner_alg_node_indigo_hydro_kinetic.o 

CPP_DEPS += \
./src/akp_local_planner_alg.d \
./src/akp_local_planner_alg_indigo_hydro_kinetic.d \
./src/akp_local_planner_alg_node.d \
./src/akp_local_planner_alg_node_indigo_hydro_kinetic.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp src/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


