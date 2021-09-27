CFLAGS =  -e -D__ICC_VERSION=80401 -D__BUILD=0 -DATtiny84  -l -A -g -MEnhanced -Wa-W 
ASFLAGS = $(CFLAGS) 
LFLAGS =  -cross_module_type_checking -g -nb:0 -e:0x2000 -Wl-W -bfunc_lit:0x22.0x2000 -dram_end:0x25f -bdata:0x60.0x25f -dhwstk_size:20 -beeprom:0.512 -fihx_coff -S2
FILES = main.o 

default:	$(FILES)
	$(CC) -o default $(LFLAGS) @..\ATTINY84A_Prueba\Programa_ATTINY84A.lk  -lcavr
