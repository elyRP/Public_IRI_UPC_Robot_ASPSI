################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include src/test/subdir.mk
-include src/scene_elements/subdir.mk
-include src/random/subdir.mk
-include src/nav/subdir.mk
-include src/learning_parameters/subdir.mk
-include src/examples/subdir.mk
-include src/subdir.mk
-include build/src/examples/CMakeFiles/prediction_example.dir/subdir.mk
-include build/src/CMakeFiles/people_prediction.dir/scene_elements/subdir.mk
-include build/src/CMakeFiles/people_prediction.dir/random/subdir.mk
-include build/src/CMakeFiles/people_prediction.dir/nav/subdir.mk
-include build/src/CMakeFiles/people_prediction.dir/learning_parameters/subdir.mk
-include build/src/CMakeFiles/people_prediction.dir/subdir.mk
-include build/CMakeFiles/2.8.12.2/CompilerIdCXX/subdir.mk
-include build/CMakeFiles/2.8.12.2/CompilerIdC/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: ASSAOP_local_lib

# Tool invocations
ASSAOP_local_lib: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: GCC C++ Linker'
	g++  -o "ASSAOP_local_lib" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(OBJS)$(C++_DEPS)$(C_DEPS)$(CC_DEPS)$(CPP_DEPS)$(EXECUTABLES)$(CXX_DEPS)$(C_UPPER_DEPS) ASSAOP_local_lib
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
