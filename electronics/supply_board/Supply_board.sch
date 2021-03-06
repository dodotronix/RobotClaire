EESchema Schematic File Version 5
EELAYER 32 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
Comment5 ""
Comment6 ""
Comment7 ""
Comment8 ""
Comment9 ""
$EndDescr
Connection ~ 5225 2175
Connection ~ 5225 1675
Connection ~ 6025 2175
Connection ~ 5225 2975
Connection ~ 5225 2475
Connection ~ 6025 2975
Wire Wire Line
	2525 1650 2775 1650
Wire Wire Line
	2575 3925 2575 3800
Wire Wire Line
	2675 3125 2950 3125
Wire Wire Line
	2700 2150 2775 2150
Wire Wire Line
	2700 2200 2700 2150
Wire Wire Line
	2775 3750 2775 3925
Wire Wire Line
	2775 3750 2975 3750
Wire Wire Line
	2975 3750 2975 3825
Wire Wire Line
	3225 3925 3225 3800
Wire Wire Line
	3425 3750 3425 3925
Wire Wire Line
	3425 3750 3625 3750
Wire Wire Line
	3625 3750 3625 3825
Wire Wire Line
	3750 3025 3950 3025
Wire Wire Line
	3750 3125 3875 3125
Wire Wire Line
	3875 1650 4150 1650
Wire Wire Line
	3875 2150 3950 2150
Wire Wire Line
	3875 3125 3875 3150
Wire Wire Line
	3950 2150 3950 2200
Wire Wire Line
	4100 3925 4100 3625
Wire Wire Line
	4200 3750 4200 3925
Wire Wire Line
	4200 3750 4400 3750
Wire Wire Line
	4400 3750 4400 3825
Wire Wire Line
	4875 3925 4875 3625
Wire Wire Line
	4975 3750 4975 3925
Wire Wire Line
	4975 3750 5175 3750
Wire Wire Line
	5125 1675 5225 1675
Wire Wire Line
	5125 2175 5225 2175
Wire Wire Line
	5125 2475 5225 2475
Wire Wire Line
	5125 2975 5225 2975
Wire Wire Line
	5175 3750 5175 3825
Wire Wire Line
	5225 1675 5600 1675
Wire Wire Line
	5225 2175 5450 2175
Wire Wire Line
	5225 2475 5600 2475
Wire Wire Line
	5225 2975 5450 2975
Wire Wire Line
	5600 1675 5600 1875
Wire Wire Line
	5600 2475 5600 2675
Wire Wire Line
	5650 3750 5850 3750
Wire Wire Line
	5650 3925 5650 3750
Wire Wire Line
	5850 2175 6025 2175
Wire Wire Line
	5850 2975 6025 2975
Wire Wire Line
	5850 3750 5850 3825
Wire Wire Line
	6025 1675 6125 1675
Wire Wire Line
	6025 2175 6125 2175
Wire Wire Line
	6025 2475 6125 2475
Wire Wire Line
	6025 2975 6125 2975
Wire Wire Line
	6325 3625 6325 3900
Text GLabel 2775 1750 0    47   Input ~ 0
INT_A1
Text GLabel 2775 1850 0    47   Input ~ 0
INT_B1
Text GLabel 2775 1950 0    47   Input ~ 0
INT_A0
Text GLabel 2775 2050 0    47   Input ~ 0
INT_B0
Text GLabel 2950 2725 0    47   Input ~ 0
INT_B0
Text GLabel 2950 2825 0    47   Input ~ 0
INT_A0
Text GLabel 2950 2925 0    47   Input ~ 0
INT_B1
Text GLabel 2950 3025 0    47   Input ~ 0
INT_A1
Text GLabel 3750 2725 2    47   Input ~ 0
MTR0
Text GLabel 3750 2825 2    47   Input ~ 0
MTR1
Text GLabel 3750 2925 2    47   Input ~ 0
Light
Text GLabel 3875 1750 2    47   Input ~ 0
ENC_A1
Text GLabel 3875 1850 2    47   Input ~ 0
ENC_B1
Text GLabel 3875 1950 2    47   Input ~ 0
ENC_A0
Text GLabel 3875 2050 2    47   Input ~ 0
ENC_B0
Text GLabel 3900 3925 1    47   Input ~ 0
ENC_B0
Text GLabel 4000 3925 1    47   Input ~ 0
ENC_A0
Text GLabel 4675 3925 1    47   Input ~ 0
ENC_B1
Text GLabel 4775 3925 1    47   Input ~ 0
ENC_A1
Text GLabel 5125 2175 0    47   Input ~ 0
MTR0
Text GLabel 5125 2975 0    47   Input ~ 0
MTR1
Text GLabel 5450 3925 1    47   Input ~ 0
MTR0_H
Text GLabel 5550 3925 1    47   Input ~ 0
MTR1_H
Text GLabel 6125 2175 2    47   Input ~ 0
MTR0_H
Text GLabel 6125 2975 2    47   Input ~ 0
MTR1_H
Text GLabel 6125 3900 1    47   Input ~ 0
Light
$Comp
L Supply_board-rescue:+3V3 #PWR1
U 1 1 56AEC48B
P 2525 1650
F 0 "#PWR1" H 2525 1600 40  0001 C CNN
F 1 "+3V3" V 2575 1800 40  0000 C CNN
F 2 "" H 2525 1650 60  0000 C CNN
F 3 "" H 2525 1650 60  0000 C CNN
	1    2525 1650
	0    -1   -1   0   
$EndComp
$Comp
L Supply_board-rescue:+3V3 #PWR3
U 1 1 56AEE41F
P 2675 3125
F 0 "#PWR3" H 2675 3075 40  0001 C CNN
F 1 "+3V3" V 2725 3300 40  0000 C CNN
F 2 "" H 2675 3125 60  0000 C CNN
F 3 "" H 2675 3125 60  0000 C CNN
	1    2675 3125
	0    -1   -1   0   
$EndComp
$Comp
L Supply_board-rescue:+3V3 #PWR15
U 1 1 56AE15BA
P 5125 1675
F 0 "#PWR15" H 5125 1625 40  0001 C CNN
F 1 "+3V3" V 5175 1850 40  0000 C CNN
F 2 "" H 5125 1675 60  0000 C CNN
F 3 "" H 5125 1675 60  0000 C CNN
	1    5125 1675
	0    -1   -1   0   
$EndComp
$Comp
L Supply_board-rescue:+3V3 #PWR16
U 1 1 56AE2408
P 5125 2475
F 0 "#PWR16" H 5125 2425 40  0001 C CNN
F 1 "+3V3" V 5175 2650 40  0000 C CNN
F 2 "" H 5125 2475 60  0000 C CNN
F 3 "" H 5125 2475 60  0000 C CNN
	1    5125 2475
	0    -1   -1   0   
$EndComp
$Comp
L power:+12V #PWR11
U 1 1 56AECE7E
P 2575 3800
F 0 "#PWR11" H 2575 3750 20  0001 C CNN
F 1 "+12V" H 2600 3950 30  0000 C CNN
F 2 "" H 2575 3800 60  0000 C CNN
F 3 "" H 2575 3800 60  0000 C CNN
	1    2575 3800
	1    0    0    -1  
$EndComp
$Comp
L power:+12V #PWR2
U 1 1 56AEBD4F
P 3225 3800
F 0 "#PWR2" H 3225 3750 20  0001 C CNN
F 1 "+12V" H 3250 3950 30  0000 C CNN
F 2 "" H 3225 3800 60  0000 C CNN
F 3 "" H 3225 3800 60  0000 C CNN
	1    3225 3800
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR?
U 1 1 56AE114A
P 3950 3025
F 0 "#PWR?" H 3925 2975 60  0001 C CNN
F 1 "+5V" V 3900 3150 40  0000 C CNN
F 2 "" H 3950 3025 60  0000 C CNN
F 3 "" H 3950 3025 60  0000 C CNN
	1    3950 3025
	0    1    1    0   
$EndComp
$Comp
L power:+5V #PWR?
U 1 1 56AE1466
P 4100 3625
F 0 "#PWR?" H 4075 3575 60  0001 C CNN
F 1 "+5V" H 4125 3775 40  0000 C CNN
F 2 "" H 4100 3625 60  0000 C CNN
F 3 "" H 4100 3625 60  0000 C CNN
	1    4100 3625
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR?
U 1 1 56AE0D2D
P 4150 1650
F 0 "#PWR?" H 4125 1600 60  0001 C CNN
F 1 "+5V" V 4100 1775 40  0000 C CNN
F 2 "" H 4150 1650 60  0000 C CNN
F 3 "" H 4150 1650 60  0000 C CNN
	1    4150 1650
	0    1    1    0   
$EndComp
$Comp
L power:+5V #PWR?
U 1 1 56AE150C
P 4875 3625
F 0 "#PWR?" H 4850 3575 60  0001 C CNN
F 1 "+5V" H 4900 3775 40  0000 C CNN
F 2 "" H 4875 3625 60  0000 C CNN
F 3 "" H 4875 3625 60  0000 C CNN
	1    4875 3625
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR?
U 1 1 56AE0EDF
P 6125 1675
F 0 "#PWR?" H 6100 1625 60  0001 C CNN
F 1 "+5V" V 6075 1800 40  0000 C CNN
F 2 "" H 6125 1675 60  0000 C CNN
F 3 "" H 6125 1675 60  0000 C CNN
	1    6125 1675
	0    1    1    0   
$EndComp
$Comp
L power:+5V #PWR?
U 1 1 56AE0F53
P 6125 2475
F 0 "#PWR?" H 6100 2425 60  0001 C CNN
F 1 "+5V" V 6075 2600 40  0000 C CNN
F 2 "" H 6125 2475 60  0000 C CNN
F 3 "" H 6125 2475 60  0000 C CNN
	1    6125 2475
	0    1    1    0   
$EndComp
$Comp
L power:+5V #PWR?
U 1 1 56AE1544
P 6325 3625
F 0 "#PWR?" H 6300 3575 60  0001 C CNN
F 1 "+5V" H 6350 3775 40  0000 C CNN
F 2 "" H 6325 3625 60  0000 C CNN
F 3 "" H 6325 3625 60  0000 C CNN
	1    6325 3625
	1    0    0    -1  
$EndComp
$Comp
L Supply_board-rescue:GND #PWR?
U 1 1 56AE0D17
P 2700 2300
F 0 "#PWR?" H 2750 2350 60  0001 C CNN
F 1 "GND" H 2700 2250 40  0000 C CNN
F 2 "" H 2700 2300 60  0000 C CNN
F 3 "" H 2700 2300 60  0000 C CNN
	1    2700 2300
	1    0    0    -1  
$EndComp
$Comp
L Supply_board-rescue:GND #PWR?
U 1 1 56AE119A
P 2975 3925
F 0 "#PWR?" H 3025 3975 60  0001 C CNN
F 1 "GND" H 2975 3875 40  0000 C CNN
F 2 "" H 2975 3925 60  0000 C CNN
F 3 "" H 2975 3925 60  0000 C CNN
	1    2975 3925
	1    0    0    -1  
$EndComp
$Comp
L Supply_board-rescue:GND #PWR?
U 1 1 56AE105A
P 3625 3925
F 0 "#PWR?" H 3675 3975 60  0001 C CNN
F 1 "GND" H 3625 3875 40  0000 C CNN
F 2 "" H 3625 3925 60  0000 C CNN
F 3 "" H 3625 3925 60  0000 C CNN
	1    3625 3925
	1    0    0    -1  
$EndComp
$Comp
L Supply_board-rescue:GND #PWR?
U 1 1 56AE0FC6
P 3875 3250
F 0 "#PWR?" H 3925 3300 60  0001 C CNN
F 1 "GND" H 3875 3200 40  0000 C CNN
F 2 "" H 3875 3250 60  0000 C CNN
F 3 "" H 3875 3250 60  0000 C CNN
	1    3875 3250
	1    0    0    -1  
$EndComp
$Comp
L Supply_board-rescue:GND #PWR?
U 1 1 56AE0E8E
P 3950 2300
F 0 "#PWR?" H 4000 2350 60  0001 C CNN
F 1 "GND" H 3950 2250 40  0000 C CNN
F 2 "" H 3950 2300 60  0000 C CNN
F 3 "" H 3950 2300 60  0000 C CNN
	1    3950 2300
	1    0    0    -1  
$EndComp
$Comp
L Supply_board-rescue:GND #PWR?
U 1 1 56AE10C9
P 4400 3925
F 0 "#PWR?" H 4450 3975 60  0001 C CNN
F 1 "GND" H 4400 3875 40  0000 C CNN
F 2 "" H 4400 3925 60  0000 C CNN
F 3 "" H 4400 3925 60  0000 C CNN
	1    4400 3925
	1    0    0    -1  
$EndComp
$Comp
L Supply_board-rescue:GND #PWR?
U 1 1 56AE11E1
P 5175 3925
F 0 "#PWR?" H 5225 3975 60  0001 C CNN
F 1 "GND" H 5175 3875 40  0000 C CNN
F 2 "" H 5175 3925 60  0000 C CNN
F 3 "" H 5175 3925 60  0000 C CNN
	1    5175 3925
	1    0    0    -1  
$EndComp
$Comp
L Supply_board-rescue:GND #PWR?
U 1 1 56AE1228
P 5850 3925
F 0 "#PWR?" H 5900 3975 60  0001 C CNN
F 1 "GND" H 5850 3875 40  0000 C CNN
F 2 "" H 5850 3925 60  0000 C CNN
F 3 "" H 5850 3925 60  0000 C CNN
	1    5850 3925
	1    0    0    -1  
$EndComp
$Comp
L Supply_board-rescue:R R1
U 1 1 56AE0ECA
P 5225 1925
F 0 "R1" H 5300 2100 40  0000 C CNN
F 1 "10K" V 5232 1926 40  0000 C CNN
F 2 "" V 5155 1925 30  0000 C CNN
F 3 "" H 5225 1925 30  0000 C CNN
	1    5225 1925
	1    0    0    -1  
$EndComp
$Comp
L Supply_board-rescue:R R2
U 1 1 56AE23FC
P 5225 2725
F 0 "R2" H 5300 2900 40  0000 C CNN
F 1 "10K" V 5232 2726 40  0000 C CNN
F 2 "" V 5155 2725 30  0000 C CNN
F 3 "" H 5225 2725 30  0000 C CNN
	1    5225 2725
	1    0    0    -1  
$EndComp
$Comp
L Supply_board-rescue:R R3
U 1 1 56AE0F59
P 6025 1925
F 0 "R3" H 6100 2100 40  0000 C CNN
F 1 "10K" V 6032 1926 40  0000 C CNN
F 2 "" V 5955 1925 30  0000 C CNN
F 3 "" H 6025 1925 30  0000 C CNN
	1    6025 1925
	1    0    0    -1  
$EndComp
$Comp
L Supply_board-rescue:R R4
U 1 1 56AE2402
P 6025 2725
F 0 "R4" H 6100 2900 40  0000 C CNN
F 1 "10K" V 6032 2726 40  0000 C CNN
F 2 "" V 5955 2725 30  0000 C CNN
F 3 "" H 6025 2725 30  0000 C CNN
	1    6025 2725
	1    0    0    -1  
$EndComp
$Comp
L Supply_board-rescue:CONN_2 P6
U 1 1 56AECE78
P 2675 4275
F 0 "P6" V 2625 4275 40  0000 C CNN
F 1 "SABBER" V 2725 4275 40  0000 C CNN
F 2 "my_footprints:XT30" H 2675 4275 60  0001 C CNN
F 3 "" H 2675 4275 60  0000 C CNN
	1    2675 4275
	0    -1   1    0   
$EndComp
$Comp
L Supply_board-rescue:CONN_2 P1
U 1 1 56090123
P 3325 4275
F 0 "P1" V 3275 4275 40  0000 C CNN
F 1 "BATT" V 3375 4275 40  0000 C CNN
F 2 "my_footprints:XT30" H 3325 4275 60  0001 C CNN
F 3 "" H 3325 4275 60  0000 C CNN
	1    3325 4275
	0    1    1    0   
$EndComp
$Comp
L Supply_board-rescue:CONN_3 K1
U 1 1 560901EC
P 5550 4275
F 0 "K1" V 5500 4275 50  0000 C CNN
F 1 "SIG_MTR" V 5600 4275 40  0000 C CNN
F 2 "w_pin_strip:pin_strip_3" H 5550 4275 60  0001 C CNN
F 3 "" H 5550 4275 60  0000 C CNN
	1    5550 4275
	0    1    1    0   
$EndComp
$Comp
L Supply_board-rescue:CONN_2 P8
U 1 1 56095641
P 6225 4250
F 0 "P8" V 6175 4250 40  0000 C CNN
F 1 "LED_SIG" V 6275 4250 40  0000 C CNN
F 2 "w_pin_strip:pin_strip_2" H 6225 4250 60  0001 C CNN
F 3 "" H 6225 4250 60  0000 C CNN
	1    6225 4250
	0    1    1    0   
$EndComp
$Comp
L Supply_board-rescue:BSS138 Q1
U 1 1 56AE00F3
P 5650 2075
F 0 "Q1" V 5875 2075 40  0000 R CNN
F 1 "BSS138" V 5500 2300 40  0000 R CNN
F 2 "SOT-23" H 5520 2177 29  0001 C CNN
F 3 "" H 5650 2075 60  0000 C CNN
	1    5650 2075
	0    1    1    0   
$EndComp
$Comp
L Supply_board-rescue:BSS138 Q2
U 1 1 56AE23F6
P 5650 2875
F 0 "Q2" V 5875 2900 40  0000 R CNN
F 1 "BSS138" V 5500 3100 40  0000 R CNN
F 2 "SOT-23" H 5520 2977 29  0001 C CNN
F 3 "" H 5650 2875 60  0000 C CNN
	1    5650 2875
	0    1    1    0   
$EndComp
$Comp
L Supply_board-rescue:CONN_4 P4
U 1 1 560916E7
P 4050 4275
F 0 "P4" V 4000 4275 50  0000 C CNN
F 1 "ENK0" V 4100 4275 50  0000 C CNN
F 2 "w_pin_strip:pin_strip_4" H 4050 4275 60  0001 C CNN
F 3 "" H 4050 4275 60  0000 C CNN
	1    4050 4275
	0    1    1    0   
$EndComp
$Comp
L Supply_board-rescue:CONN_4 P7
U 1 1 56090227
P 4825 4275
F 0 "P7" V 4775 4275 50  0000 C CNN
F 1 "ENK1" V 4875 4275 50  0000 C CNN
F 2 "w_pin_strip:pin_strip_4" H 4825 4275 60  0001 C CNN
F 3 "" H 4825 4275 60  0000 C CNN
	1    4825 4275
	0    1    1    0   
$EndComp
$Comp
L Supply_board-rescue:CONN_6 P2
U 1 1 5648D4E4
P 3125 1900
F 0 "P2" V 3075 1900 60  0000 C CNN
F 1 "LV_SHIFT" V 3175 1900 60  0000 C CNN
F 2 "w_pin_strip:pin_socket_6" H 3125 1900 60  0001 C CNN
F 3 "" H 3125 1900 60  0000 C CNN
	1    3125 1900
	1    0    0    -1  
$EndComp
$Comp
L Supply_board-rescue:CONN_6 P5
U 1 1 5648D539
P 3525 1900
F 0 "P5" V 3475 1900 60  0000 C CNN
F 1 "LV_SHIFT" V 3575 1900 60  0000 C CNN
F 2 "w_pin_strip:pin_socket_6" H 3525 1900 60  0001 C CNN
F 3 "" H 3525 1900 60  0000 C CNN
	1    3525 1900
	-1   0    0    -1  
$EndComp
$Comp
L Supply_board-rescue:CONN_5X2 P3
U 1 1 5627CFCA
P 3350 2925
F 0 "P3" H 3350 3225 60  0000 C CNN
F 1 "SIGNALS" V 3350 2925 50  0000 C CNN
F 2 "w_conn_strip:vasch_strip_5x2" H 3350 2925 60  0001 C CNN
F 3 "" H 3350 2925 60  0000 C CNN
	1    3350 2925
	1    0    0    -1  
$EndComp
$EndSCHEMATC
