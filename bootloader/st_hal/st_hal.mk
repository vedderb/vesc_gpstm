# ST HAL files.
STHALSRC = st_hal/stm32g4xx_hal_flash.c \
	st_hal/stm32g4xx_hal_flash_ex.c \
	st_hal/stm32g4xx_hal_flash_ramfunc.c \
	st_hal/stm32g4xx_ll_tim.c \
	st_hal/hal_functions.c

STHALINC = st_hal

# Shared variables
ALLCSRC += $(STHALSRC)
ALLINC  += $(STHALINC)
