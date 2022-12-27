all: libopencm3/lib/libopencm3_stm32f1.a example

clean:
	make -C src clean

libopencm3/lib/libopencm3_stm32f1.a:
	make -C libopencm3

example:
	make -C src clean && make -C src

.PHONY: all clean