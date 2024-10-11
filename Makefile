##############################################################################
# Build global options
# NOTE: Can be overridden externally.
#

ifndef GIT_COMMIT_HASH
  GIT_COMMIT_HASH := $(shell git rev-parse --short HEAD)
endif

ifndef GIT_BRANCH_NAME
  GIT_BRANCH_NAME := $(shell git rev-parse --abbrev-ref HEAD)
endif

ifdef HW_SRC
  ifndef HW_HEADER
    $(error HW_HEADER not defined while HW_SRC was set, you must set both!)
  endif
  USE_CUSTOM_HW := 1
endif
ifdef HW_HEADER
  ifndef HW_SRC
    $(error HW_SRC not defined while HW_HEADER was set, you must set both!)
  endif
  USE_CUSTOM_HW := 1
endif

# Compiler options here.
ifeq ($(USE_OPT),)
  USE_OPT = -O2 -ggdb -fomit-frame-pointer -falign-functions=16 -D_GNU_SOURCE
  USE_OPT += -DGIT_COMMIT_HASH=\"$(GIT_COMMIT_HASH)\" -DGIT_BRANCH_NAME=\"$(GIT_BRANCH_NAME)\"
  ifdef USER_GIT_COMMIT_HASH
    USE_OPT += -DUSER_GIT_COMMIT_HASH=\"$(USER_GIT_COMMIT_HASH)\"
  endif
  ifdef USER_GIT_BRANCH_NAME
    USE_OPT += -DUSER_GIT_BRANCH_NAME=\"$(USER_GIT_BRANCH_NAME)\"
  endif
  
  ifneq ($(USE_CUSTOM_HW),)
    USE_OPT += -DHW_SOURCE="\"$(HW_SRC)\"" -DHW_HEADER="\"$(HW_HEADER)\""
  endif
  
  USE_OPT += $(build_args)
  USE_OPT += -fsingle-precision-constant -Wdouble-promotion
endif

# C specific options here (added to USE_OPT).
ifeq ($(USE_COPT),)
  USE_COPT = 
endif

# C++ specific options here (added to USE_OPT).
ifeq ($(USE_CPPOPT),)
  USE_CPPOPT = -fno-rtti
endif

# Enable this if you want the linker to remove unused code and data.
ifeq ($(USE_LINK_GC),)
  USE_LINK_GC = yes
endif

# Linker extra options here.
ifeq ($(USE_LDOPT),)
  USE_LDOPT = 
endif

# Enable this if you want link time optimizations (LTO).
ifeq ($(USE_LTO),)
  USE_LTO = yes
endif

# Enable this if you want to see the full log while compiling.
ifeq ($(USE_VERBOSE_COMPILE),)
  USE_VERBOSE_COMPILE = yes
endif

# If enabled, this option makes the build process faster by not compiling
# modules not used in the current configuration.
ifeq ($(USE_SMART_BUILD),)
  USE_SMART_BUILD = yes
endif

#
# Build global options
##############################################################################

##############################################################################
# Architecture or project specific options
#

# Stack size to be allocated to the Cortex-M process stack. This stack is
# the stack used by the main() thread.
ifeq ($(USE_PROCESS_STACKSIZE),)
  USE_PROCESS_STACKSIZE = 0x400
endif

# Stack size to the allocated to the Cortex-M main/exceptions stack. This
# stack is used for processing interrupts and exceptions.
ifeq ($(USE_EXCEPTIONS_STACKSIZE),)
  USE_EXCEPTIONS_STACKSIZE = 0x400
endif

# Enables the use of FPU (no, softfp, hard).
ifeq ($(USE_FPU),)
  USE_FPU = hard
endif

# FPU-related options.
ifeq ($(USE_FPU_OPT),)
  USE_FPU_OPT = -mfloat-abi=$(USE_FPU) -mfpu=fpv4-sp-d16
endif

#
# Architecture or project specific options
##############################################################################

##############################################################################
# Project, target, sources and paths
#

# Define project name here
PROJECT = vesc_gpstm

# Target settings.
MCU  = cortex-m4

# Imported source files and paths.
CHIBIOS  := ChibiOS_21.11.3
CONFDIR  := ./cfg
BUILDDIR := ./build
DEPDIR   := ./.dep

# Licensing files.
include $(CHIBIOS)/os/license/license.mk
# Startup files.
include $(CHIBIOS)/os/common/startup/ARMCMx/compilers/GCC/mk/startup_stm32g4xx.mk
# HAL-OSAL files (optional).
include $(CHIBIOS)/os/hal/hal.mk
include $(CHIBIOS)/os/hal/ports/STM32/STM32G4xx/platform.mk
include board/board.mk
include $(CHIBIOS)/os/hal/osal/rt-nil/osal.mk
# RTOS files (optional).
include $(CHIBIOS)/os/rt/rt.mk
include $(CHIBIOS)/os/common/ports/ARMv7-M/compilers/GCC/mk/port.mk
# Auto-build files in ./source recursively.
include $(CHIBIOS)/tools/mk/autobuild.mk
include st_hal/st_hal.mk

# Define linker script file here
LDSCRIPT= STM32G431xB.ld

# C sources that can be compiled in ARM or THUMB mode depending on the global
# setting.
CSRC = $(ALLCSRC) \
       main.c \
       utils.c \
       comm_can.c \
       crc.c \
       hwconf/hw.c \
       buffer.c \
       commands.c \
       packet.c \
       config/confparser.c \
       config/confxml.c \
       terminal.c \
       flash_helper.c \
       conf_general.c \
       lwprintf/lwprintf.c \
       timer.c \
       timeout.c

# C++ sources that can be compiled in ARM or THUMB mode depending on the global
# setting.
CPPSRC = $(ALLCPPSRC)

# List ASM source files here.
ASMSRC = $(ALLASMSRC)

# List ASM with preprocessor source files here.
ASMXSRC = $(ALLXASMSRC)

# Inclusion directories.
INCDIR = $(CONFDIR) $(ALLINC) hwconf config st_hal lwprintf

# Define C warning options here.
CWARN = -Wall -Wextra -Wundef -Wstrict-prototypes

# Define C++ warning options here.
CPPWARN = -Wall -Wextra -Wundef

#
# Project, target, sources and paths
##############################################################################

##############################################################################
# Start of user section
#

# List all user C define here, like -D_DEBUG=1
UDEFS =

# Define ASM defines here
UADEFS =

# List all user directories here
UINCDIR =

# List the user directory to look for the libraries here
ULIBDIR =

# List all user libraries here
ULIBS = -lm --specs=nosys.specs

#
# End of user section
##############################################################################

##############################################################################
# Common rules
#

RULESPATH = $(CHIBIOS)/os/common/startup/ARMCMx/compilers/GCC/mk
include $(RULESPATH)/arm-none-eabi.mk
include $(RULESPATH)/rules.mk

SRC_SENTINEL := $(BUILDDIR)/hw_src
HEADER_SENTINEL := $(BUILDDIR)/hw_header

# Update the tracker files if HW_SRC or HW_HEADER has changed to trigger a rebuild
ifeq ($(shell if [[ -d $(BUILDDIR) ]]; then printf 1; else printf ""; fi),1)
  ifneq ($(file < $(SRC_SENTINEL)),$(HW_SRC))
    $(info Updated $(SRC_SENTINEL))
    $(file > $(SRC_SENTINEL),$(HW_SRC))
    $(shell touch hwconf/hw.c conf_general.h)
  endif
  ifneq ($(file < $(HEADER_SENTINEL)),$(HW_HEADER))
    $(info Updated $(HEADER_SENTINEL))
    $(file > $(HEADER_SENTINEL),$(HW_HEADER))
    $(shell touch hwconf/hw.h conf_general.h)
  endif
endif

upload: build/$(PROJECT).bin
	openocd -f stm32g4_stlinkv2.cfg \
		-c "program build/$(PROJECT).elf verify reset exit"
