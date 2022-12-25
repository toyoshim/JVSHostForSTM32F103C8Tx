all: libopencm3/lib/libopencm3_stm32f1.a src/host_example.bin

libopencm3/lib/libopencm3_stm32f1.a:
	make -C libopencm3

src/host_example.bin:
	make -C src