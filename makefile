include lpc1768.mk

OBJS = lpc1768_startup.o lpc1768_handlers.o system_LPC17xx.o \
	   core_cm3.o retrieve_eui64.o sbrk.o lpc17xx_pinsel.o \
	   lpc17xx_i2c.o lpc17xx_clkpwr.o
OUTPUT = euitest

all:$(OUTPUT).bin

lpc1768_startup.o: 
	$(CC) $(CFLAGS) lpc1768_startup.c -o $@ 

lpc1768_handlers.o : 
	$(CC) $(CFLAGS) lpc1768_handlers.c -o $@

system_LPC17xx.o:
	$(CC) $(CFLAGS) system_LPC17xx.c -o $@

core_cm3.o: 
	$(CC) $(CFLAGS) core_cm3.c -o $@
	
lpc17xx_pinsel.o:
	$(CC) $(CFLAGS) lpc17xx_pinsel.c -o $@
	
lpc17xx_clkpwr.o:
	$(CC) $(CFLAGS) lpc17xx_clkpwr.c -o $@
	
lpc17xx_i2c.o:
	$(CC) $(CFLAGS) lpc17xx_i2c.c -o $@

sbrk.o:
	$(CC) $(CFLAGS) sbrk.c -o $@

retrieve_eui64.o:
	${CC} ${CFLAGS} retrieve_eui64.c -o $@

$(OUTPUT).bin: $(OBJS)
	$(LD) $(LDFLAGS) -T $(LD_SCRIPT) -o $(OUTPUT).elf $(OBJS) -lm 
	$(OBJCOPY) $(OCFLAGS) -O ihex $(OUTPUT).elf $(OUTPUT).hex
	$(OBJCOPY) $(OCFLAGS) -O binary $(OUTPUT).elf $(OUTPUT).bin
	#$(LPCRC) $(OUTPUT).elf
	#$(LPCRC) $(OUTPUT).hex
	#$(LPCRC) $(OUTPUT).bin
clean:
	rm -f *.o $(OUTPUT).elf $(OUTPUT).bin $(OUTPUT).hex
