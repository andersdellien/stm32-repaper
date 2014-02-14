CC      = arm-none-eabi-gcc
LD      = arm-none-eabi-gcc
AR      = arm-none-eabi-ar
AS      = arm-none-eabi-as
CP      = arm-none-eabi-objcopy
OD	= arm-none-eabi-objdump
  
CFLAGS = -I. -Ist-lib/STM32F4xx_StdPeriph_Driver/inc
CFLAGS += -Ist-lib/CMSIS/ST/STM32F4xx/Include
CFLAGS += -Ist-lib/CMSIS/Include
CFLAGS += -c -fno-common -O0 -g -mcpu=cortex-m4 -mthumb 
AFLAGS  = -ahls -mapcs-32 -o crt.o
LFLAGS  = -Tstm32f4.ld -nostartfiles
CPFLAGS = -Obinary
ODFLAGS	= -S

BUILD_DIR := out

SRCS := \
	main-repaper.c \
	venus_1_44.c \
	aphrodite_1_44.c \
	system_stm32f4xx.c \
	st-lib/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_rcc.c \
	st-lib/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_gpio.c \
	st-lib/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_usart.c \
	st-lib/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_tim.c \
	st-lib/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_i2c.c \
	st-lib/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_spi.c

OBJS := ${SRCS:%.c=${BUILD_DIR}/%.o}

all: test

clean:
	-rm main.list $(OBJS) main.out main.hex main.map 

test: main.out
	@ echo "...copying"
	$(CP) $(CPFLAGS) main.out main.bin
	$(OD) $(ODFLAGS) main.out > main.list

main.out: $(OBJS) blink-ram.cmd 
	@echo "..linking"
	$(LD) $(LFLAGS) -o main.out $(OBJS)

${BUILD_DIR}/%.o:%.c 
	mkdir -p $(dir $@)
	@echo "Compiling..." $<
	$(CC) $(CFLAGS) -o $@ $<
	 
