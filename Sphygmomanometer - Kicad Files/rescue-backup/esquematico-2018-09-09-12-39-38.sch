EESchema Schematic File Version 2
LIBS:esquematico-rescue
LIBS:power
LIBS:device
LIBS:switches
LIBS:relays
LIBS:motors
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
LIBS:ads8867
LIBS:mp3v5050gp
LIBS:ths4521
LIBS:ad7124
LIBS:24cw640
LIBS:edu-ciaa-nxp
LIBS:hc-06
LIBS:lm2575-adj
LIBS:mic5319-3V3YD5
LIBS:mp3v5050
LIBS:tl7660
LIBS:lcd16x2-i2c
LIBS:esquematico-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 3
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L MP3V5050GP U1
U 1 1 5B44F578
P 2150 3175
F 0 "U1" H 2325 3475 60  0000 C CNN
F 1 "MP3V5050GP" H 2550 3575 60  0000 C CNN
F 2 "" H 2150 3175 60  0001 C CNN
F 3 "" H 2150 3175 60  0001 C CNN
	1    2150 3175
	1    0    0    -1  
$EndComp
$Comp
L C C2
U 1 1 5B44F5BE
P 1700 3225
F 0 "C2" H 1725 3325 50  0000 L CNN
F 1 "1.0uF" H 1725 3125 50  0000 L CNN
F 2 "" H 1738 3075 50  0001 C CNN
F 3 "" H 1700 3225 50  0001 C CNN
F 4 "CL10B105KP8NNNC" H 1700 3225 60  0001 C CNN "manuf number"
	1    1700 3225
	1    0    0    -1  
$EndComp
$Comp
L C C1
U 1 1 5B44F5E9
P 1350 3225
F 0 "C1" H 1375 3325 50  0000 L CNN
F 1 "0.1uF" H 1375 3125 50  0000 L CNN
F 2 "" H 1388 3075 50  0001 C CNN
F 3 "" H 1350 3225 50  0001 C CNN
F 4 "CL10F104ZO8NNNC" H 1350 3225 60  0001 C CNN "manuf number"
	1    1350 3225
	1    0    0    -1  
$EndComp
$Comp
L C C3
U 1 1 5B44F60C
P 2650 3375
F 0 "C3" H 2675 3475 50  0000 L CNN
F 1 "470pF" H 2675 3275 50  0000 L CNN
F 2 "" H 2688 3225 50  0001 C CNN
F 3 "" H 2650 3375 50  0001 C CNN
F 4 "CL10C471JB8NNNC" H 2650 3375 60  0001 C CNN "manuf number"
	1    2650 3375
	1    0    0    -1  
$EndComp
$Comp
L GNDA #PWR4
U 1 1 5B44F65F
P 2150 3875
F 0 "#PWR4" H 2150 3625 50  0001 C CNN
F 1 "GNDA" H 2150 3725 50  0000 C CNN
F 2 "" H 2150 3875 50  0001 C CNN
F 3 "" H 2150 3875 50  0001 C CNN
	1    2150 3875
	1    0    0    -1  
$EndComp
$Comp
L GNDA #PWR5
U 1 1 5B44F67D
P 2650 3875
F 0 "#PWR5" H 2650 3625 50  0001 C CNN
F 1 "GNDA" H 2650 3725 50  0000 C CNN
F 2 "" H 2650 3875 50  0001 C CNN
F 3 "" H 2650 3875 50  0001 C CNN
	1    2650 3875
	1    0    0    -1  
$EndComp
$Comp
L GNDA #PWR2
U 1 1 5B44F694
P 1700 3875
F 0 "#PWR2" H 1700 3625 50  0001 C CNN
F 1 "GNDA" H 1700 3725 50  0000 C CNN
F 2 "" H 1700 3875 50  0001 C CNN
F 3 "" H 1700 3875 50  0001 C CNN
	1    1700 3875
	1    0    0    -1  
$EndComp
$Comp
L GNDA #PWR1
U 1 1 5B44F6AB
P 1350 3875
F 0 "#PWR1" H 1350 3625 50  0001 C CNN
F 1 "GNDA" H 1350 3725 50  0000 C CNN
F 2 "" H 1350 3875 50  0001 C CNN
F 3 "" H 1350 3875 50  0001 C CNN
	1    1350 3875
	1    0    0    -1  
$EndComp
Wire Wire Line
	2650 3875 2650 3525
Wire Wire Line
	2650 3225 2650 3175
Wire Wire Line
	2150 3625 2150 3875
Wire Wire Line
	1700 3875 1700 3375
Wire Wire Line
	1350 3875 1350 3375
Wire Wire Line
	1350 3075 1350 2625
Wire Wire Line
	2150 2525 2150 2725
Wire Wire Line
	1700 3075 1700 2625
Connection ~ 1700 2625
$Comp
L VDDA #PWR3
U 1 1 5B44F709
P 2150 2525
F 0 "#PWR3" H 2150 2375 50  0001 C CNN
F 1 "VDDA" H 2150 2675 50  0000 C CNN
F 2 "" H 2150 2525 50  0001 C CNN
F 3 "" H 2150 2525 50  0001 C CNN
	1    2150 2525
	1    0    0    -1  
$EndComp
Connection ~ 2150 2625
Connection ~ 2650 3175
Wire Wire Line
	1350 2625 2150 2625
Wire Wire Line
	2600 3175 3150 3175
$Comp
L AD7124 U3
U 1 1 5B44FA4A
P 7000 3800
F 0 "U3" H 8575 4475 60  0000 C CNN
F 1 "AD7124" H 6000 4475 60  0000 C CNN
F 2 "" H 6625 3775 60  0001 C CNN
F 3 "" H 6625 3775 60  0001 C CNN
	1    7000 3800
	1    0    0    -1  
$EndComp
$Comp
L C C7
U 1 1 5B44FAE3
P 6900 4825
F 0 "C7" H 6925 4925 50  0000 L CNN
F 1 "0.1uF" H 6925 4725 50  0000 L CNN
F 2 "" H 6938 4675 50  0001 C CNN
F 3 "" H 6900 4825 50  0001 C CNN
F 4 "CL10F104ZO8NNNC" H 6900 4825 60  0001 C CNN "manuf number"
	1    6900 4825
	1    0    0    -1  
$EndComp
$Comp
L C C8
U 1 1 5B44FB63
P 8075 4825
F 0 "C8" H 8100 4925 50  0000 L CNN
F 1 "0.1uF" H 8100 4725 50  0000 L CNN
F 2 "" H 8113 4675 50  0001 C CNN
F 3 "" H 8075 4825 50  0001 C CNN
F 4 "CL10F104ZO8NNNC" H 8075 4825 60  0001 C CNN "manuf number"
	1    8075 4825
	1    0    0    -1  
$EndComp
$Comp
L GNDA #PWR11
U 1 1 5B44FBC5
P 6700 4675
F 0 "#PWR11" H 6700 4425 50  0001 C CNN
F 1 "GNDA" H 6700 4525 50  0000 C CNN
F 2 "" H 6700 4675 50  0001 C CNN
F 3 "" H 6700 4675 50  0001 C CNN
	1    6700 4675
	1    0    0    -1  
$EndComp
$Comp
L GNDA #PWR12
U 1 1 5B44FC00
P 6900 5025
F 0 "#PWR12" H 6900 4775 50  0001 C CNN
F 1 "GNDA" H 6900 4875 50  0000 C CNN
F 2 "" H 6900 5025 50  0001 C CNN
F 3 "" H 6900 5025 50  0001 C CNN
	1    6900 5025
	1    0    0    -1  
$EndComp
Wire Wire Line
	6900 5025 6900 4975
Wire Wire Line
	6900 4675 6900 4625
Wire Wire Line
	6700 4675 6700 4625
Wire Wire Line
	8075 4675 8075 4625
$Comp
L GNDD #PWR13
U 1 1 5B44FCD7
P 8075 5000
F 0 "#PWR13" H 8075 4750 50  0001 C CNN
F 1 "GNDD" H 8075 4875 50  0000 C CNN
F 2 "" H 8075 5000 50  0001 C CNN
F 3 "" H 8075 5000 50  0001 C CNN
	1    8075 5000
	1    0    0    -1  
$EndComp
Wire Wire Line
	8075 5000 8075 4975
$Comp
L GNDD #PWR15
U 1 1 5B44FD54
P 8275 4650
F 0 "#PWR15" H 8275 4400 50  0001 C CNN
F 1 "GNDD" H 8275 4525 50  0000 C CNN
F 2 "" H 8275 4650 50  0001 C CNN
F 3 "" H 8275 4650 50  0001 C CNN
	1    8275 4650
	1    0    0    -1  
$EndComp
Wire Wire Line
	8275 4650 8275 4625
$Comp
L VDDA #PWR10
U 1 1 5B44FE9F
P 6675 2900
F 0 "#PWR10" H 6675 2750 50  0001 C CNN
F 1 "VDDA" H 6675 3050 50  0000 C CNN
F 2 "" H 6675 2900 50  0001 C CNN
F 3 "" H 6675 2900 50  0001 C CNN
	1    6675 2900
	1    0    0    -1  
$EndComp
Wire Wire Line
	6675 2900 6675 2975
$Comp
L VDD #PWR14
U 1 1 5B44FF67
P 8275 2900
F 0 "#PWR14" H 8275 2750 50  0001 C CNN
F 1 "VDD" H 8275 3050 50  0000 C CNN
F 2 "" H 8275 2900 50  0001 C CNN
F 3 "" H 8275 2900 50  0001 C CNN
	1    8275 2900
	1    0    0    -1  
$EndComp
Wire Wire Line
	8275 2900 8275 2975
$Sheet
S 8950 3150 1425 1225
U 5B44FFE9
F0 "MicroControlador" 60
F1 "uC.sch" 60
F2 "SPI_SCK" I L 8950 3850 60 
F3 "SPI_MISO" I L 8950 3550 60 
F4 "SPI_MOSI" I L 8950 3700 60 
F5 "SPI_CS" I L 8950 4000 60 
F6 "~SYNC" I L 8950 4275 60 
$EndSheet
Wire Wire Line
	8950 3550 8850 3550
Wire Wire Line
	8950 3700 8850 3700
Wire Wire Line
	8950 3850 8850 3850
Wire Wire Line
	8950 4000 8850 4000
NoConn ~ 7425 4625
NoConn ~ 8850 3325
$Comp
L GNDA #PWR9
U 1 1 5B491C02
P 5500 4350
F 0 "#PWR9" H 5500 4100 50  0001 C CNN
F 1 "GNDA" H 5500 4200 50  0000 C CNN
F 2 "" H 5500 4350 50  0001 C CNN
F 3 "" H 5500 4350 50  0001 C CNN
	1    5500 4350
	1    0    0    -1  
$EndComp
Wire Wire Line
	5500 4350 5500 3425
Wire Wire Line
	5500 3425 5600 3425
Wire Wire Line
	5600 3575 5500 3575
Connection ~ 5500 3575
Wire Wire Line
	5600 3725 5500 3725
Connection ~ 5500 3725
Wire Wire Line
	5600 3875 5500 3875
Connection ~ 5500 3875
Wire Wire Line
	5600 4175 5500 4175
Connection ~ 5500 4175
Wire Wire Line
	5600 4325 5500 4325
Connection ~ 5500 4325
Wire Wire Line
	5600 4025 5500 4025
Connection ~ 5500 4025
$Comp
L GNDA #PWR8
U 1 1 5B520EB7
P 4900 3875
F 0 "#PWR8" H 4900 3625 50  0001 C CNN
F 1 "GNDA" H 4900 3725 50  0000 C CNN
F 2 "" H 4900 3875 50  0001 C CNN
F 3 "" H 4900 3875 50  0001 C CNN
	1    4900 3875
	1    0    0    -1  
$EndComp
$Comp
L GNDA #PWR6
U 1 1 5B520F0A
P 4150 3875
F 0 "#PWR6" H 4150 3625 50  0001 C CNN
F 1 "GNDA" H 4150 3725 50  0000 C CNN
F 2 "" H 4150 3875 50  0001 C CNN
F 3 "" H 4150 3875 50  0001 C CNN
	1    4150 3875
	1    0    0    -1  
$EndComp
Wire Wire Line
	4900 3875 4900 3575
$Comp
L VDDA #PWR7
U 1 1 5B521025
P 4900 2925
F 0 "#PWR7" H 4900 2775 50  0001 C CNN
F 1 "VDDA" H 4900 3075 50  0000 C CNN
F 2 "" H 4900 2925 50  0001 C CNN
F 3 "" H 4900 2925 50  0001 C CNN
	1    4900 2925
	1    0    0    -1  
$EndComp
Wire Wire Line
	4900 2925 4900 2975
Wire Wire Line
	8850 4275 8950 4275
NoConn ~ 7625 2975
NoConn ~ 7475 2975
NoConn ~ 7200 2975
Wire Wire Line
	5300 3275 5600 3275
Wire Wire Line
	4700 3375 4625 3375
Wire Wire Line
	4625 3375 4625 3550
Wire Wire Line
	4625 3550 5350 3550
Wire Wire Line
	5350 3550 5350 2600
Connection ~ 5350 3275
$Comp
L R R1
U 1 1 5B524430
P 3300 3175
F 0 "R1" V 3380 3175 50  0000 C CNN
F 1 "33K" V 3300 3175 50  0000 C CNN
F 2 "" V 3230 3175 50  0001 C CNN
F 3 "" H 3300 3175 50  0001 C CNN
	1    3300 3175
	0    1    1    0   
$EndComp
$Comp
L R R2
U 1 1 5B52449A
P 3825 3175
F 0 "R2" V 3905 3175 50  0000 C CNN
F 1 "33K" V 3825 3175 50  0000 C CNN
F 2 "" V 3755 3175 50  0001 C CNN
F 3 "" H 3825 3175 50  0001 C CNN
	1    3825 3175
	0    1    1    0   
$EndComp
$Comp
L C_Small C5
U 1 1 5B524531
P 4150 3425
F 0 "C5" H 4160 3495 50  0000 L CNN
F 1 "10nF" H 4160 3345 50  0000 L CNN
F 2 "" H 4150 3425 50  0001 C CNN
F 3 "" H 4150 3425 50  0001 C CNN
F 4 "06035C103KAT2A" H 4150 3425 60  0001 C CNN "manuf number"
	1    4150 3425
	1    0    0    -1  
$EndComp
$Comp
L C_Small C4
U 1 1 5B5245A7
P 4125 2600
F 0 "C4" H 4135 2670 50  0000 L CNN
F 1 "10nF" H 4135 2520 50  0000 L CNN
F 2 "" H 4125 2600 50  0001 C CNN
F 3 "" H 4125 2600 50  0001 C CNN
	1    4125 2600
	0    1    1    0   
$EndComp
Wire Wire Line
	3975 3175 4700 3175
Wire Wire Line
	4150 3325 4150 3175
Connection ~ 4150 3175
Wire Wire Line
	4150 3875 4150 3525
Wire Wire Line
	3675 3175 3450 3175
Wire Wire Line
	3575 3175 3575 2600
Wire Wire Line
	3575 2600 4025 2600
Connection ~ 3575 3175
Wire Wire Line
	5350 2600 4225 2600
$Comp
L MCP6002-xSN U2
U 1 1 5B8D8DA2
P 5000 3275
F 0 "U2" H 5000 3475 50  0000 L CNN
F 1 "MCP6002-xSN" H 5000 3075 50  0000 L CNN
F 2 "" H 5000 3275 50  0001 C CNN
F 3 "" H 5000 3275 50  0001 C CNN
	1    5000 3275
	1    0    0    -1  
$EndComp
$EndSCHEMATC