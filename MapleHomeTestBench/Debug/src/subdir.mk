################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/DeviceConnetor.c \
../src/FilterComb.c \
../src/FilterCommonMode.c \
../src/FilterHighPass.c \
../src/MapleDebuggAppn.c \
../src/rms.c \
../src/xy_plot.c 

OBJS += \
./src/DeviceConnetor.o \
./src/FilterComb.o \
./src/FilterCommonMode.o \
./src/FilterHighPass.o \
./src/MapleDebuggAppn.o \
./src/rms.o \
./src/xy_plot.o 

C_DEPS += \
./src/DeviceConnetor.d \
./src/FilterComb.d \
./src/FilterCommonMode.d \
./src/FilterHighPass.d \
./src/MapleDebuggAppn.d \
./src/rms.d \
./src/xy_plot.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -I/usr/include/atk-1.0 -I/usr/include/mysql -I/usr/include/cairo -I/usr/include/pixman-1 -I/usr/include/freetype2 -I/usr/include/libpng12 -I/usr/include/gdk-pixbuf-2.0 -I/usr/include/glib-2.0 -I/usr/include/gtk-3.0 -I/usr/include/at-spi2-atk/2.0 -I/usr/include/at-spi-2.0 -I/usr/include/dbus-1.0 -I/usr/include/gio-unix-2.0/ -I/usr/include/mirclient -I/usr/include/mircore -I/usr/include/mircookie -I/usr/include/cairo -I/usr/include/pango-1.0 -I/usr/include/harfbuzz -I/usr/lib/x86_64-linux-gnu/dbus-1.0/include -I/usr/lib/x86_64-linux-gnu/glib-2.0/include -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


