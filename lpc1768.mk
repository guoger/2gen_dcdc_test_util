CC       = arm-none-eabi-gcc -lm -g -O0 
AS       = arm-none-eabi-as
AR       = arm-none-eabi-ar -r
LD       = arm-none-eabi-gcc -lm
NM       = arm-none-eabi-nm
OBJDUMP  = arm-none-eabi-objdump
OBJCOPY  = arm-none-eabi-objcopy
READELF  = arm-none-eabi-readelf
CODESIZE = arm-none-eabi-size


CPU       		= cortex-m3
#OPTIM			= 0
OPTIM			= 2

#===================== C compile flag ============================
CFLAGS    		= -c 
CFLAGS			+= -mcpu=$(CPU) 
CFLAGS			+= -mthumb 
CFLAGS			+= -Wall 
CFLAGS			+= -O$(OPTIM) 
CFLAGS			+= -mapcs-frame 
CFLAGS			+= -D__thumb2__=1 
CFLAGS	 		+= -msoft-float 
CFLAGS			+= -gdwarf-2 
CFLAGS   		+= -mno-sched-prolog 
CFLAGS			+= -fno-hosted 
CFLAGS			+= -mtune=cortex-m3 
CFLAGS			+= -march=armv7-m 
CFLAGS			+= -mfix-cortex-m3-ldrd  
CFLAGS   		+= -ffunction-sections 
CFLAGS			+= -fdata-sections 
CFLAGS			+= -lm 

#================= note =================================
#CFLAGS			+= -mthumb-interwork  
#CFLAGS			+= -mno-bit-align 
#CFLAGS			+= mstructure-size-boundary=8
#CFLAGS			+= -Wpacked
#CFLAGS			+= -Wpadded
#CFLAGS			+= -fpack-struct=0 

#================ note2 ================================
#CFLAGS			+= -fno-builtin
#CFLAGS			+= -fno-strict-aliasing  
#CFLAGS			+= -D PACK_STRUCT_END=__attribute\(\(packed\)\)
#CFLAGS			+= -D ALIGN_STRUCT_END=__attribute\(\(aligned\(4\)\)\)
#CFLAGS			+= -fmessage-length=0 
#CFLAGS			+= -funsigned-char 
#CFLAGS			+= -Wextra 
#CFLAGS			+= -MMD 
#CFLAGS			+= -MP 
#CFLAGS			+= -MF"$(@:%.o=%.d)" 
#CFLAGS			+= -MT"$(@:%.o=%.d)" 

#================ Build Folder Include ========================
#CFLAGS   		+= -I$(FWLIB_INC_DIR) -I$(CMCORE_INC_DIR) #-I$(DEVICE_INC_DIR)



#================ Asm compile flag ========================
AFLAGS    		= -mcpu=$(CPU) 
#AFLAGS   		+= -I$(FWLIB_INC_DIR) -I$(CMCORE_INC_DIR) #-I$(DEVICE_INC_DIR) 
AFLAGS   		+= -gdwarf-2 

#ASFLAGS = -c -g -Os $(INCLUDE_PATHS) -Wall -mthumb -ffunction-sections #-fdata-sections -fmessage-length=0 -mcpu=$(CPU_TYPE) -D__ASSEMBLY__ -x #assembler-with-cpp


LDFLAGS =  -nostartfiles -static -mcpu=cortex-m3 -mthumb -mthumb-interwork -Wl,--gc-sections -lm
OCFLAGS = --strip-unneeded 
LD_SCRIPT = cortex-m3.ld
LPCRC = etc/lpcrc
