CXX = g++-4.7
CXXFLAGS = -std=c++11 -O3
LDLIBS = -lchdl
ARCH = 4w8/8/1

#dump.vcd : harmonica-vl
#	./harmonica-vl

#harmonica-vl : harmonica.v testbench.v vgacont.v
#	iverilog -o harmonica-vl harmonica.v testbench.v vgacont.v

#harmonica.v : chdl_design.v
#	sed 's/chdl_design/harmonica/' < chdl_design.v > harmonica.v

#chdl_design.v : harmonica.nand
#	nand2v < harmonica.nand > chdl_design.v

harmonica.nand harmonica.vcd : harmonica rom.hex
	./harmonica

harmonica: harmonica.o pipeline.o

harmonica.o : harmonica.cpp pipeline.h funcunit.h harpinst.h \
              regfile.h fpu.h
pipeline.o  : pipeline.cpp pipeline.h

rom.hex : rom.bin
	hexdump -v -e '1/4 "%08x" "\n"' rom.bin > rom.hex

rom.bin : rom.HOF
	harptool -L --arch $(ARCH) -o rom.bin rom.HOF

rom.HOF : rom.s
	harptool -A --arch $(ARCH) -o rom.HOF rom.s

clean:
	rm -f harmonica harmonica-vl harmonica.nand harmonica.vcd dump.vcd *~ \
              harmonica.v chdl_design.v *.o rom.hex rom.bin rom.HOF
