################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../local_lib_people_prediction/src/iri_geometry.cpp \
../local_lib_people_prediction/src/prediction_behavior.cpp \
../local_lib_people_prediction/src/prediction_bhmip.cpp \
../local_lib_people_prediction/src/scene_abstract.cpp \
../local_lib_people_prediction/src/scene_sim.cpp 

OBJS += \
./local_lib_people_prediction/src/iri_geometry.o \
./local_lib_people_prediction/src/prediction_behavior.o \
./local_lib_people_prediction/src/prediction_bhmip.o \
./local_lib_people_prediction/src/scene_abstract.o \
./local_lib_people_prediction/src/scene_sim.o 

CPP_DEPS += \
./local_lib_people_prediction/src/iri_geometry.d \
./local_lib_people_prediction/src/prediction_behavior.d \
./local_lib_people_prediction/src/prediction_bhmip.d \
./local_lib_people_prediction/src/scene_abstract.d \
./local_lib_people_prediction/src/scene_sim.d 


# Each subdirectory must supply rules for building sources it contributes
local_lib_people_prediction/src/%.o: ../local_lib_people_prediction/src/%.cpp local_lib_people_prediction/src/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


