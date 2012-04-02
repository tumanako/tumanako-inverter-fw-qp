

################################################################################
#   Tumanako_QP - Electric Vehicle and Motor control software
#
#   Copyright (C) 2002-2011 Quantum Leaps, LLC. All rights reserved.
#   Copyright (C) 2012 Bernard Mentink <bmentink@gmail.com>
#
#   This file is part of Tumanako_QP.
#
#   Tumanako_QP is free software: you can redistribute it and/or modify
#   it under the terms of the GNU Lesser General Public License as published
#   by the Free Software Foundation, either version 3 of the License, or
#   (at your option) any later version.
#
#   Tumanako_QP is distributed in the hope that it will be useful,
#   but WITHOUT ANY WARRANTY; without even the implied warranty of
#   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#   GNU Lesser General Public License for more details.
#
#   You should have received a copy of the GNU Lesser General Public License
#   along with Tumanako_QP.  If not, see <http:#www.gnu.org/licenses/>.
# ------------------------------------
# examples of invoking this Makefile:
# building configurations: Debug (default), Release
# make
# make CONF=rel
#
# cleaning configurations: Debug (default), Release
# make clean
# make CONF=rel clean
################################################################################


#-----------------------------------------------------------------------------
# NOTE: the Makefile expects that the QPCPP environment variable is defined
# and points to the QP/C++ installation directory
#
ifndef QPCPP
QPCPP := /home/bmentink/QP/qpcpp
endif

#-----------------------------------------------------------------------------
# general utilities
#

RM    := rm -rf
MKDIR := mkdir

#-----------------------------------------------------------------------------
# tools
#
ifeq ($(GNU_ARM),)
#GNU_ARM = /usr/local/bin/arm-none-eabi
GNU_ARM=/home/bmentink/ARM_Tools/arm-2010q1/bin/arm-none-eabi
#GNU_ARM=/home/bmentink/ARM_Tools/arm-2011.09/bin/arm-none-eabi
#GNU_ARM = /usr/local/lpcxpresso_4.2.0_224/tools/bin/arm-none-eabi
#GNU_ARM=/home/bmentink/devkitPro/devkitARM/bin/arm-eabi
#GNU_ARM=/usr/local/redsuite_4.2.0_225/redsuite/tools/bin/arm-none-eabi
endif

CC    := $(GNU_ARM)-gcc
CPP   := $(GNU_ARM)-g++
AS    := $(GNU_ARM)-as
LINK  := $(GNU_ARM)-g++                   # for C++ programs
BIN   := $(GNU_ARM)-objcopy
SIZE  := $(GNU_ARM)-size
DUMP  := $(GNU_ARM)-objdump


#-----------------------------------------------------------------------------
# directories
#
QP_PORT_DIR := $(QPCPP)/ports/arm-cortex/qk/gnu
OPENCM3_DIR := ../libopencm3
APP_DIR     := .
LIB_DIR     :=

# source directories
VPATH = $(APP_DIR) \
	$(QPCPP)/source \
	$(OPENCM3_DIR)/lib/stm32 \
	./IFOC 

# Output file basename
OUTPUT    := tumanako_vc_sine

# include directories
INCLUDES  = -I$(QPCPP)/include	\
	-I$(QP_PORT_DIR) \
	-I$(OPENCM3_DIR)/include \
	-I./IFOC \
	-Iinclude \
	-I. 


# defines
DEFINES = -D__NEWLIB__

#-----------------------------------------------------------------------------
# files
#

# assembler source files
ASM_SRCS :=

# C source files
C_SRCS := $(wildcard *.c) 


# C++ source files
CPP_SRCS := $(wildcard *.cpp)

LD_SCRIPT := $(OUTPUT).ld

#-----------------------------------------------------------------------------
# build options for various configurations
#

ARM_CORE = cortex-m3

ifeq (rel, $(CONF))       # Release configuration ............................

BIN_DIR := rel
LIBS    := -lqk_$(ARM_CORE)_dk -lqf_$(ARM_CORE)_dk -lqep_$(ARM_CORE)_dk  -lopencm3_stm32f1 

ASFLAGS = -mcpu=$(ARM_CORE)

CFLAGS = -mcpu=$(ARM_CORE) -mthumb -Wall \
	-Os $(INCLUDES) $(DEFINES) -DNDEBUG  
	
CPPFLAGS = -mcpu=$(ARM_CORE) -mthumb -Wall \
	-fno-rtti -fno-exceptions  \
	-Os $(INCLUDES) $(DEFINES) -DNDEBUG 

LINKFLAGS = -T$(LD_SCRIPT) -nostartfiles  \
	-Wl,--gc-sections,-Map=$(BIN_DIR)/$(OUTPUT).map   \
	-mthumb -march=armv7 -L$(QP_PORT_DIR)/$(BIN_DIR) -L$(OPENCM3_DIR)/lib/stm32/f1 -mfix-cortex-m3-ldrd -msoft-float  

else                     # default Debug configuration .......................

BIN_DIR := dbg
LIBS    := -lqk_$(ARM_CORE)_dk -lqf_$(ARM_CORE)_dk -lqep_$(ARM_CORE)_dk  -lopencm3_stm32f1

ASFLAGS = -g -mcpu=$(ARM_CORE)

CFLAGS = -mcpu=$(ARM_CORE) -mthumb -Wall \
	-g -O $(INCLUDES) $(DEFINES)
	
CPPFLAGS = -mcpu=$(ARM_CORE) -mthumb -Wall \
	-fno-rtti -fno-exceptions  \
	-g -O $(INCLUDES) $(DEFINES)

		 	
LINKFLAGS = -T$(LD_SCRIPT) -nostartfiles  \
	-Wl,--gc-sections,-Map=$(BIN_DIR)/$(OUTPUT).map,-cref \
	-mthumb -march=armv7 -L$(QP_PORT_DIR)/$(BIN_DIR) -L$(OPENCM3_DIR)/lib/stm32/f1 -mfix-cortex-m3-ldrd -msoft-float 
	
endif


ASM_OBJS     := $(patsubst %.s,%.o,$(ASM_SRCS))
C_OBJS       := $(patsubst %.c,%.o,$(C_SRCS))
CPP_OBJS     := $(patsubst %.cpp,%.o,$(CPP_SRCS))

TARGET_BIN   := $(BIN_DIR)/$(OUTPUT).bin
TARGET_HEX   := $(BIN_DIR)/$(OUTPUT).hex
TARGET_ELF   := $(BIN_DIR)/$(OUTPUT).elf
TARGET_LIST   := $(BIN_DIR)/$(OUTPUT).list
ASM_OBJS_EXT := $(addprefix $(BIN_DIR)/, $(ASM_OBJS))
C_OBJS_EXT   := $(addprefix $(BIN_DIR)/, $(C_OBJS))
C_DEPS_EXT   := $(patsubst %.o, %.d, $(C_OBJS_EXT))
CPP_OBJS_EXT := $(addprefix $(BIN_DIR)/, $(CPP_OBJS))
CPP_DEPS_EXT := $(patsubst %.o, %.d, $(CPP_OBJS_EXT))


#-----------------------------------------------------------------------------
# rules
#

# Uncomment what target format you need ....
#all: $(BIN_DIR) $(TARGET_ELF)
all: $(BIN_DIR) $(TARGET_HEX)
#all: $(BIN_DIR) $(TARGET_BIN)

$(BIN_DIR):
	@echo
	mkdir -p $@

$(TARGET_BIN): $(TARGET_ELF)
	$(BIN) -O binary $< $@
	
$(TARGET_HEX): $(TARGET_ELF)
	$(BIN) -O ihex $< $@

$(TARGET_ELF) : $(ASM_OBJS_EXT) $(C_OBJS_EXT) $(CPP_OBJS_EXT)
	$(LINK) $(LINKFLAGS) -o $@ $^ $(LIBS)
	$(SIZE) $@
	$(DUMP) -S $(TARGET_ELF) > $(TARGET_LIST)

$(BIN_DIR)/%.d : %.c
	$(CC) -MM -MT $(@:.d=.o) $(CFLAGS) $< > $@

$(BIN_DIR)/%.d : %.cpp
	$(CPP) -MM -MT $(@:.d=.o) $(CPPFLAGS) $< > $@

$(BIN_DIR)/%.o : %.s
	$(AS) $(ASFLAGS) $< -o $@

$(BIN_DIR)/%.o : %.c
	$(CC) $(CFLAGS) -c $< -o $@

$(BIN_DIR)/%.o : %.cpp
	$(CPP) $(CPPFLAGS) -c $< -o $@

-include $(C_DEPS_EXT) $(CPP_DEPS_EXT)


.PHONY : clean
clean:
	-$(RM) $(BIN_DIR)/*.o \
	$(BIN_DIR)/*.d \
	$(BIN_DIR)/*.bin \
	$(BIN_DIR)/*.elf \
	$(BIN_DIR)/*.hex \
	$(BIN_DIR)/*.map
	
show:
	@echo CONF = $(CONF)
	@echo ASM_SRCS = $(ASM_SRCS)
	@echo C_SRCS = $(C_SRCS)
	@echo CPP_SRCS = $(CPP_SRCS)
	@echo ASM_OBJS_EXT = $(ASM_OBJS_EXT)
	@echo C_OBJS_EXT = $(C_OBJS_EXT)
	@echo C_DEPS_EXT = $(C_DEPS_EXT)
	@echo CPP_DEPS_EXT = $(CPP_DEPS_EXT)
	@echo TARGET_ELF = $(TARGET_ELF)