PROJECT = host_example
BUILD_DIR = bin

SHARED_DIR = ../jvsio
CFILES = main.c
CFILES += JVSIO_c.c
CFILES += jvs_host.c

# TODO - you will need to edit these two lines!
DEVICE=stm32f103c8tx
OOCD_FILE = board/stm32f103c8_blue_pill.cfg

# You shouldn't have to edit anything below here.
VPATH += $(SHARED_DIR)
INCLUDES += $(patsubst %,-I%, . $(SHARED_DIR))
OPENCM3_DIR=../libopencm3

include $(OPENCM3_DIR)/mk/genlink-config.mk
include ../rules.mk
include $(OPENCM3_DIR)/mk/genlink-rules.mk
