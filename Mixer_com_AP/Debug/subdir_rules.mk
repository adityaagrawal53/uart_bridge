################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
%.obj: ../%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Arm Compiler - building file: "$<"'
	"/home/paz/Downloads/ti-cgt-arm_18.12.2.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me --include_path="/home/paz/ti/ccs2050/ccs/ccs_base/arm/include" --include_path="/home/paz/ti/ccs2050/ccs/ccs_base/arm/include/CMSIS" --include_path="/home/paz/workspace_ccstheia/Mixer_com_AP" --include_path="/home/paz/workspace_ccstheia/Mixer_com_AP/include" --include_path="/home/paz/workspace_ccstheia/Mixer_com_AP/include/driverlib" --include_path="/home/paz/workspace_ccstheia/Mixer_com_AP/include/driverlib/MSP432P4xx" --include_path="/home/paz/workspace_ccstheia/Mixer_com_AP/src" --include_path="/home/paz/workspace_ccstheia/Mixer_com_AP/include/driverlib" --include_path="/home/paz/workspace_ccstheiaMixer_com_AP/include/driverlib/MSP432P4xx" --include_path="/home/paz/workspace_ccstheia/Mixer_com_AP/src" --include_path="/home/paz/Downloads/ti-cgt-arm_18.12.2.LTS/include" --advice:power=all --define=__MSP432P401R__ --define=ccs -g --gcc --diag_warning=225 --diag_wrap=off --display_error_number --abi=eabi --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$(shell echo $<)"
	@echo 'Finished building: "$<"'
	@echo ' '


