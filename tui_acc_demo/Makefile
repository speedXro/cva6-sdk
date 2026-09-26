APP_COMPILER = /tools/repos/cva6-sdk-binutils-239/buildroot/output/host/bin/riscv64-buildroot-linux-gnu-gcc

CFLAGS = -Wall -O2
# To use the RISC-V `cycle` CSR instead of clock_gettime for timing
# (only works if the kernel enabled user access via scounteren.CY):
#   make CFLAGS="-Wall -O2 -DUSE_RDCYCLE"

.PHONY: all clean

all: clean tuiacc_demo

tuiacc_demo: tuiacc_demo.o dma.o SWDCT2DQ.o stft_ref.o
	$(APP_COMPILER) $(CFLAGS) tuiacc_demo.o dma.o SWDCT2DQ.o stft_ref.o -o tuiacc_demo

tuiacc_demo.o: tuiacc_demo.c dma.h SWDCT2DQ.h stft_ref.h
	$(APP_COMPILER) $(CFLAGS) -c tuiacc_demo.c

stft_ref.o: stft_ref.c stft_ref.h stft_coeffs.h
	$(APP_COMPILER) $(CFLAGS) -c stft_ref.c

SWDCT2DQ.o: SWDCT2DQ.c SWDCT2DQ.h
	$(APP_COMPILER) $(CFLAGS) -c SWDCT2DQ.c

dma.o: dma.c dma.h
	$(APP_COMPILER) $(CFLAGS) -c dma.c

clean:
	@rm -f *.o tuiacc_demo
