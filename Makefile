# MPLAB IDE generated this makefile for use with GNU make.
# Project: SOTA.mcp
# Date: Mon Nov 11 15:16:20 2024

AS = xc16-as.exe
CC = xc16-gcc.exe
LD = xc16-ld.exe
AR = xc16-ar.exe
HX = xc16-bin2hex.exe
RM = rm

SOTA.hex : SOTA.cof
	$(HX) "SOTA.cof" -omf=coff

SOTA.cof : Objects-SOTA/adc.o Objects-SOTA/advolts.o Objects-SOTA/i2c1.o Objects-SOTA/pid.o Objects-SOTA/pwm.o Objects-SOTA/timer1.o Objects-SOTA/timer23.o Objects-SOTA/traps.o Objects-SOTA/main.o Objects-SOTA/TASKBlowers.o Objects-SOTA/TASKHeater.o Objects-SOTA/TASKInterfacePhoenix.o Objects-SOTA/TASKSerialPhoenix.o Objects-SOTA/uart1.o
	$(CC) -omf=coff -mcpu=33FJ256GP710 "Objects-SOTA\adc.o" "Objects-SOTA\advolts.o" "Objects-SOTA\i2c1.o" "Objects-SOTA\pid.o" "Objects-SOTA\pwm.o" "Objects-SOTA\timer1.o" "Objects-SOTA\timer23.o" "Objects-SOTA\traps.o" "Objects-SOTA\main.o" "Objects-SOTA\TASKBlowers.o" "Objects-SOTA\TASKHeater.o" "Objects-SOTA\TASKInterfacePhoenix.o" "Objects-SOTA\TASKSerialPhoenix.o" "Objects-SOTA\uart1.o" -o"SOTA.cof" -Wl,-L"C:\Program Files\Microchip\MPLAB C30\lib",--script="SOTA\p33FJ256GP710PS-TASK.gld",--defsym=__MPLAB_BUILD=1,-Map="SOTA.map",--report-mem

Objects-SOTA/adc.o : SAGE/adc.h SOTA/p33FJ256GP710.h SAGE/adc.c
	$(CC) -omf=coff -mcpu=33FJ256GP710 -x c -c "SAGE\adc.c" -o"Objects-SOTA\adc.o" -I"." -I"SOTA" -I"SAGE" -I"TCPIP_App" -I"Microchip\Include" -I"Salvo\Inc" -I"Salvo\Inc\MCC30" -g -Wall

Objects-SOTA/advolts.o : SAGE/advolts.h SAGE/adc.h SOTA/fire.h SOTA/TASKHeater.h SAGE/advolts.c
	$(CC) -omf=coff -mcpu=33FJ256GP710 -x c -c "SAGE\advolts.c" -o"Objects-SOTA\advolts.o" -I"." -I"SOTA" -I"SAGE" -I"TCPIP_App" -I"Microchip\Include" -I"Salvo\Inc" -I"Salvo\Inc\MCC30" -g -Wall

Objects-SOTA/i2c1.o : SAGE/i2c1.h SOTA/common.h SOTA/p33FJ256GP710.h SAGE/i2c1.c
	$(CC) -omf=coff -mcpu=33FJ256GP710 -x c -c "SAGE\i2c1.c" -o"Objects-SOTA\i2c1.o" -I"." -I"SOTA" -I"SAGE" -I"TCPIP_App" -I"Microchip\Include" -I"Salvo\Inc" -I"Salvo\Inc\MCC30" -g -Wall

Objects-SOTA/pid.o : SAGE/pid.h SAGE/pid.c
	$(CC) -omf=coff -mcpu=33FJ256GP710 -x c -c "SAGE\pid.c" -o"Objects-SOTA\pid.o" -I"." -I"SOTA" -I"SAGE" -I"TCPIP_App" -I"Microchip\Include" -I"Salvo\Inc" -I"Salvo\Inc\MCC30" -g -Wall

Objects-SOTA/pwm.o : SAGE/pwm.h SAGE/timer23.h SOTA/common.h SOTA/p33FJ256GP710.h SAGE/pwm.c
	$(CC) -omf=coff -mcpu=33FJ256GP710 -x c -c "SAGE\pwm.c" -o"Objects-SOTA\pwm.o" -I"." -I"SOTA" -I"SAGE" -I"TCPIP_App" -I"Microchip\Include" -I"Salvo\Inc" -I"Salvo\Inc\MCC30" -g -Wall

Objects-SOTA/timer1.o : c:/program\ files/microchip/xc16/v1.70/include/lega-c/yvals.h c:/program\ files/microchip/xc16/v1.70/include/lega-c/stdlib.h SOTA/fire.h SOTA/TASKPhoenix.h SOTA/fire.h SOTA/FIRESerialProtocol.h SAGE/adc.h SAGE/pwm.h SOTA/TASKBlowers.h SOTA/TASKHeater.h SOTA/TASKio.h SAGE/timer1.h SOTA/common.h SOTA/p33FJ256GP710.h SAGE/timer1.c
	$(CC) -omf=coff -mcpu=33FJ256GP710 -x c -c "SAGE\timer1.c" -o"Objects-SOTA\timer1.o" -I"." -I"SOTA" -I"SAGE" -I"TCPIP_App" -I"Microchip\Include" -I"Salvo\Inc" -I"Salvo\Inc\MCC30" -g -Wall

Objects-SOTA/timer23.o : SOTA/TASKio.h SAGE/pwm.h SAGE/timer23.h SOTA/common.h SOTA/p33FJ256GP710.h SAGE/timer23.c
	$(CC) -omf=coff -mcpu=33FJ256GP710 -x c -c "SAGE\timer23.c" -o"Objects-SOTA\timer23.o" -I"." -I"SOTA" -I"SAGE" -I"TCPIP_App" -I"Microchip\Include" -I"Salvo\Inc" -I"Salvo\Inc\MCC30" -g -Wall

Objects-SOTA/traps.o : SAGE/timer1.h SOTA/p33FJ256GP710.h SAGE/traps.c
	$(CC) -omf=coff -mcpu=33FJ256GP710 -x c -c "SAGE\traps.c" -o"Objects-SOTA\traps.o" -I"." -I"SOTA" -I"SAGE" -I"TCPIP_App" -I"Microchip\Include" -I"Salvo\Inc" -I"Salvo\Inc\MCC30" -g -Wall

Objects-SOTA/main.o : SOTA/TASKPhoenix.h SOTA/TASKHeater.h SOTA/TASKBlowers.h SAGE/uart1.h SOTA/TASKio.h SAGE/uart1.h SAGE/i2c1.h SAGE/pwm.h SAGE/adc.h SAGE/timer23.h SAGE/timer1.h SOTA/common.h SOTA/p33FJ256GP710.h SOTA/main.c
	$(CC) -omf=coff -mcpu=33FJ256GP710 -x c -c "SOTA\main.c" -o"Objects-SOTA\main.o" -I"." -I"SOTA" -I"SAGE" -I"TCPIP_App" -I"Microchip\Include" -I"Salvo\Inc" -I"Salvo\Inc\MCC30" -g -Wall

Objects-SOTA/TASKBlowers.o : SOTA/fire.h SOTA/FIRESerialProtocol.h SAGE/pwm.h SAGE/timer1.h SOTA/TASKio.h SOTA/TASKBlowers.h SOTA/common.h SOTA/p33FJ256GP710.h SOTA/TASKBlowers.c
	$(CC) -omf=coff -mcpu=33FJ256GP710 -x c -c "SOTA\TASKBlowers.c" -o"Objects-SOTA\TASKBlowers.o" -I"." -I"SOTA" -I"SAGE" -I"TCPIP_App" -I"Microchip\Include" -I"Salvo\Inc" -I"Salvo\Inc\MCC30" -g -Wall

Objects-SOTA/TASKHeater.o : SOTA/TASKPhoenix.h SAGE/pwm.h SAGE/advolts.h SAGE/adc.h SAGE/pid.h SOTA/TASKBlowers.h SAGE/timer1.h SOTA/TASKio.h SOTA/TASKHeater.h SOTA/common.h SOTA/p33FJ256GP710.h SOTA/TASKHeater.c
	$(CC) -omf=coff -mcpu=33FJ256GP710 -x c -c "SOTA\TASKHeater.c" -o"Objects-SOTA\TASKHeater.o" -I"." -I"SOTA" -I"SAGE" -I"TCPIP_App" -I"Microchip\Include" -I"Salvo\Inc" -I"Salvo\Inc\MCC30" -g -Wall

Objects-SOTA/TASKInterfacePhoenix.o : SOTA/TASKPhoenix.h SOTA/fire.h SOTA/FIRESerialProtocol.h SAGE/pwm.h SAGE/i2c1.h SAGE/advolts.h SAGE/adc.h SAGE/pid.h SOTA/TASKBlowers.h SOTA/TASKHeater.h SOTA/TASKio.h SAGE/timer23.h SAGE/timer1.h SOTA/common.h SOTA/p33FJ256GP710.h SOTA/TASKInterfacePhoenix.c
	$(CC) -omf=coff -mcpu=33FJ256GP710 -x c -c "SOTA\TASKInterfacePhoenix.c" -o"Objects-SOTA\TASKInterfacePhoenix.o" -I"." -I"SOTA" -I"SAGE" -I"TCPIP_App" -I"Microchip\Include" -I"Salvo\Inc" -I"Salvo\Inc\MCC30" -g -Wall

Objects-SOTA/TASKSerialPhoenix.o : SOTA/fire.h SOTA/FIRESerialProtocol.h SOTA/TASKPhoenix.h SAGE/uart1.h SOTA/TASKHeater.h SOTA/TASKio.h SAGE/i2c1.h SAGE/timer23.h SAGE/timer1.h SOTA/common.h SOTA/p33FJ256GP710.h SOTA/TASKSerialPhoenix.c
	$(CC) -omf=coff -mcpu=33FJ256GP710 -x c -c "SOTA\TASKSerialPhoenix.c" -o"Objects-SOTA\TASKSerialPhoenix.o" -I"." -I"SOTA" -I"SAGE" -I"TCPIP_App" -I"Microchip\Include" -I"Salvo\Inc" -I"Salvo\Inc\MCC30" -g -Wall

Objects-SOTA/uart1.o : Sage/i2c1.h SOTA/fire.h SOTA/FIRESerialProtocol.h Sage/uart1.h SOTA/common.h SOTA/p33FJ256GP710.h Sage/uart1.c
	$(CC) -omf=coff -mcpu=33FJ256GP710 -x c -c "Sage\uart1.c" -o"Objects-SOTA\uart1.o" -I"." -I"SOTA" -I"SAGE" -I"TCPIP_App" -I"Microchip\Include" -I"Salvo\Inc" -I"Salvo\Inc\MCC30" -g -Wall

clean : 
	$(RM) "Objects-SOTA\adc.o" "Objects-SOTA\advolts.o" "Objects-SOTA\i2c1.o" "Objects-SOTA\pid.o" "Objects-SOTA\pwm.o" "Objects-SOTA\timer1.o" "Objects-SOTA\timer23.o" "Objects-SOTA\traps.o" "Objects-SOTA\main.o" "Objects-SOTA\TASKBlowers.o" "Objects-SOTA\TASKHeater.o" "Objects-SOTA\TASKInterfacePhoenix.o" "Objects-SOTA\TASKSerialPhoenix.o" "Objects-SOTA\uart1.o" "SOTA.cof" "SOTA.hex"

