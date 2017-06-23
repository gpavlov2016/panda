CFLAGS += -I inc -I ../ -nostdlib -fno-builtin
CFLAGS += -Tstm32_flash.ld -ISTM32F4xx_HAL_Driver/Inc 

CC = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
OBJDUMP = arm-none-eabi-objdump

ifeq ($(RELEASE),1)
  CERT = ../../pandaextra/certs/release
  CFLAGS += "-DPANDA_SAFETY"
else
  CERT = ../certs/debug
  CFLAGS += "-DALLOW_DEBUG"
endif

MACHINE = $(shell uname -m)
OS = $(shell uname -o)

ifeq ($(OS),GNU/Linux)
  MACHINE := "$(MACHINE)-linux"
endif

# this pushes the unchangable bootstub too
all: obj/bootstub.$(PROJ_NAME).bin obj/$(PROJ_NAME).bin
	./tools/enter_download_mode.py
	./tools/dfu-util-$(MACHINE) -a 0 -s 0x08000000 -D obj/bootstub.$(PROJ_NAME).bin
	./tools/dfu-util-$(MACHINE) -a 0 -s 0x08004000 -D obj/$(PROJ_NAME).bin
	./tools/dfu-util-$(MACHINE) --reset-stm32 -a 0 -s 0x08000000

dfu: obj/bootstub.$(PROJ_NAME).bin obj/$(PROJ_NAME).bin
	./tools/dfu-util-$(MACHINE) -a 0 -s 0x08000000 -D obj/bootstub.$(PROJ_NAME).bin
	./tools/dfu-util-$(MACHINE) -a 0 -s 0x08004000 -D obj/$(PROJ_NAME).bin
	./tools/dfu-util-$(MACHINE) --reset-stm32 -a 0 -s 0x08000000

bootstub: obj/bootstub.$(PROJ_NAME).bin
	./tools/enter_download_mode.py
	./tools/dfu-util-$(MACHINE) -a 0 -s 0x08000000 -D obj/bootstub.$(PROJ_NAME).bin
	./tools/dfu-util-$(MACHINE) --reset-stm32 -a 0 -s 0x08000000

main: obj/$(PROJ_NAME).bin
	./tools/enter_download_mode.py
	./tools/dfu-util-$(MACHINE) -a 0 -s 0x08004000 -D obj/$(PROJ_NAME).bin
	./tools/dfu-util-$(MACHINE) --reset-stm32 -a 0 -s 0x08000000

ota: obj/$(PROJ_NAME).bin
	curl http://192.168.0.10/stupdate --upload-file $<

ifneq ($(wildcard ../.git/HEAD),) 
obj/gitversion.h: ../.git/HEAD ../.git/index
	echo "const uint8_t gitversion[] = \"$(shell git rev-parse HEAD)\";" > $@
else
ifneq ($(wildcard ../../.git/modules/panda/HEAD),) 
obj/gitversion.h: ../../.git/modules/panda/HEAD ../../.git/modules/panda/index
	echo "const uint8_t gitversion[] = \"$(shell git rev-parse HEAD)\";" > $@
else
obj/gitversion.h: 
	echo "const uint8_t gitversion[] = \"RELEASE\";" > $@
endif
endif

obj/cert.h: ../crypto/getcertheader.py
	../crypto/getcertheader.py ../certs/debug.pub ../certs/release.pub > $@

obj/bootstub.$(PROJ_NAME).o: bootstub.c early.h obj/cert.h spi_flasher.h
	$(CC) $(CFLAGS) -o $@ -c $<

obj/elm327.$(PROJ_NAME).o: elm327.c *.h obj/gitversion.h
	$(CC) $(CFLAGS) -o $@ -c $<

obj/spi.$(PROJ_NAME).o: spi.c *.h obj/gitversion.h
	$(CC) $(CFLAGS) -o $@ -c $<

obj/stm32f4xx_hal_can.$(PROJ_NAME).o: STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_can.c *.h obj/gitversion.h
	$(CC) $(CFLAGS) -o $@ -c $<

obj/stm32f4xx_hal_spi.$(PROJ_NAME).o: STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_spi.c *.h obj/gitversion.h
	$(CC) $(CFLAGS) -o $@ -c $<

obj/stm32f4xx_hal_dma.$(PROJ_NAME).o: STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c *.h obj/gitversion.h
	$(CC) $(CFLAGS) -o $@ -c $<

obj/stm32f4xx_hal.$(PROJ_NAME).o: STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c *.h obj/gitversion.h
	$(CC) $(CFLAGS) -o $@ -c $<

obj/stm32f4xx_hal_cortex.$(PROJ_NAME).o: stm32f4xx_hal_cortex.c *.h obj/gitversion.h
	$(CC) $(CFLAGS) -o $@ -c $<

obj/system_stm32f4xx.$(PROJ_NAME).o: system_stm32f4xx.c *.h obj/gitversion.h
	$(CC) $(CFLAGS) -o $@ -c $<

obj/main.$(PROJ_NAME).o: main.c *.h obj/gitversion.h
	$(CC) $(CFLAGS) -o $@ -c $<

# TODO(geohot): learn to use Makefiles
obj/sha.$(PROJ_NAME).o: ../crypto/sha.c
	$(CC) $(CFLAGS) -o $@ -c $<

obj/rsa.$(PROJ_NAME).o: ../crypto/rsa.c
	$(CC) $(CFLAGS) -o $@ -c $<

obj/$(STARTUP_FILE).o: $(STARTUP_FILE).s
	mkdir -p obj
	$(CC) $(CFLAGS) -o $@ -c $<

obj/$(PROJ_NAME).bin: obj/$(STARTUP_FILE).o obj/main.$(PROJ_NAME).o obj/elm327.$(PROJ_NAME).o obj/spi.$(PROJ_NAME).o obj/stm32f4xx_hal_can.$(PROJ_NAME).o obj/stm32f4xx_hal.$(PROJ_NAME).o obj/stm32f4xx_hal_cortex.$(PROJ_NAME).o obj/system_stm32f4xx.$(PROJ_NAME).o obj/stm32f4xx_hal_spi.$(PROJ_NAME).o obj/stm32f4xx_hal_dma.$(PROJ_NAME).o
  # hack
	$(CC) -Wl,--section-start,.isr_vector=0x8004000 $(CFLAGS) -o obj/$(PROJ_NAME).elf $^
	$(OBJCOPY) -v -O binary obj/$(PROJ_NAME).elf obj/code.bin
	SETLEN=1 ../crypto/sign.py obj/code.bin $@ $(CERT)

obj/bootstub.$(PROJ_NAME).bin: obj/$(STARTUP_FILE).o obj/bootstub.$(PROJ_NAME).o obj/sha.$(PROJ_NAME).o obj/rsa.$(PROJ_NAME).o obj/spi.$(PROJ_NAME).o
	$(CC) $(CFLAGS) -o obj/bootstub.$(PROJ_NAME).elf $^
	$(OBJCOPY) -v -O binary obj/bootstub.$(PROJ_NAME).elf $@
	
clean:
	rm -f obj/*

