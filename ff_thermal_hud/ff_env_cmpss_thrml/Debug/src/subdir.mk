################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
LD_SRCS += \
../src/lscript.ld 

C_SRCS += \
../src/function_prototype.c \
../src/memory_config_g.c \
../src/mlx90640_api.c \
../src/pcam_5C_cfgs.c \
../src/platform.c \
../src/xmipi_sp701_example.c 

OBJS += \
./src/function_prototype.o \
./src/memory_config_g.o \
./src/mlx90640_api.o \
./src/pcam_5C_cfgs.o \
./src/platform.o \
./src/xmipi_sp701_example.o 

C_DEPS += \
./src/function_prototype.d \
./src/memory_config_g.d \
./src/mlx90640_api.d \
./src/pcam_5C_cfgs.d \
./src/platform.d \
./src/xmipi_sp701_example.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MicroBlaze gcc compiler'
	mb-gcc -Wall -O0 -g3 -c -fmessage-length=0 -MT"$@" -IC:/sp701/mpcsi1218/sdk/design_1_wrapper/export/design_1_wrapper/sw/design_1_wrapper/standalone_microblaze_0/bspinclude/include -mlittle-endian -mcpu=v11.0 -mxl-soft-mul -mhard-float -Wl,--no-relax -ffunction-sections -fdata-sections -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


