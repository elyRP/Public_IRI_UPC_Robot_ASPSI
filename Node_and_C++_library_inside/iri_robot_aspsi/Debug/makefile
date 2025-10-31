################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include src/subdir.mk
-include local_lib_people_prediction/src/test/subdir.mk
-include local_lib_people_prediction/src/scene_elements/subdir.mk
-include local_lib_people_prediction/src/random/subdir.mk
-include local_lib_people_prediction/src/nav/subdir.mk
-include local_lib_people_prediction/src/learning_parameters/subdir.mk
-include local_lib_people_prediction/src/examples/subdir.mk
-include local_lib_people_prediction/src/subdir.mk
-include local_lib_people_prediction/build/src/examples/CMakeFiles/prediction_example.dir/subdir.mk
-include local_lib_people_prediction/build/src/CMakeFiles/people_prediction.dir/scene_elements/subdir.mk
-include local_lib_people_prediction/build/src/CMakeFiles/people_prediction.dir/random/subdir.mk
-include local_lib_people_prediction/build/src/CMakeFiles/people_prediction.dir/nav/subdir.mk
-include local_lib_people_prediction/build/src/CMakeFiles/people_prediction.dir/learning_parameters/subdir.mk
-include local_lib_people_prediction/build/src/CMakeFiles/people_prediction.dir/subdir.mk
-include local_lib_people_prediction/build/CMakeFiles/3.10.2/CompilerIdCXX/subdir.mk
-include local_lib_people_prediction/build/CMakeFiles/3.10.2/CompilerIdC/subdir.mk
-include local_lib_people_prediction/build/CMakeFiles/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := iri_robot_aspsi
BUILD_ARTIFACT_EXTENSION :=
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: main-build

# Main-build Target
main-build: iri_robot_aspsi

# Tool invocations
iri_robot_aspsi: $(OBJS) $(USER_OBJS) makefile objects.mk $(OPTIONAL_TOOL_DEPS)
	@echo 'Building target: $@'
	@echo 'Invoking: GCC C++ Linker'
	g++  -o "iri_robot_aspsi" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(CC_DEPS)$(C++_DEPS)$(EXECUTABLES)$(C_UPPER_DEPS)$(CXX_DEPS)$(OBJS)$(CPP_DEPS)$(C_DEPS) iri_robot_aspsi
	-@echo ' '

.PHONY: all clean dependents

-include ../makefile.targets
