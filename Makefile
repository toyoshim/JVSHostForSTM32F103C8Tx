all: libopencm3/lib/libopencm3_stm32f1.a src/host_example.bin

clean:
	make -C src clean

libopencm3/lib/libopencm3_stm32f1.a:
	make -C libopencm3

src/host_example.bin:
	make -C src clean && make -C src

.PHONY: all clean