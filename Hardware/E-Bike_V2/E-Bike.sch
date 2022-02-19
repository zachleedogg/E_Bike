EESchema Schematic File Version 4
LIBS:E-Bike-cache
EELAYER 26 0
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
$EndDescr
$Comp
L E-Bike:MAX17552 U1
U 1 1 60A9077E
P 2500 1200
F 0 "U1" H 2550 1275 50  0000 C CNN
F 1 "MAX17552AUB+" H 2550 1184 50  0000 C CNN
F 2 "Package_SO:MSOP-10_3x3mm_P0.5mm" H 2500 1200 50  0001 C CNN
F 3 "https://datasheets.maximintegrated.com/en/ds/MAX17552.pdf" H 2500 1200 50  0001 C CNN
F 4 "MAX17552AUB+" H 2500 1200 50  0001 C CNN "Part Number"
	1    2500 1200
	1    0    0    -1  
$EndComp
$Comp
L Device:L L1
U 1 1 60A90D43
P 3450 1400
F 0 "L1" V 3640 1400 50  0000 C CNN
F 1 "150u" V 3549 1400 50  0000 C CNN
F 2 "Inductor_SMD:L_Bourns-SRN4018" H 3450 1400 50  0001 C CNN
F 3 "https://www.bourns.com/docs/Product-Datasheets/SRN4026.pdf" H 3450 1400 50  0001 C CNN
F 4 "SRN4026-151M" H 3450 1400 50  0001 C CNN "Part Number"
	1    3450 1400
	0    -1   -1   0   
$EndComp
$Comp
L Device:C C5
U 1 1 60A918AE
P 4050 1550
F 0 "C5" H 4165 1596 50  0000 L CNN
F 1 "10u" H 4165 1505 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 4088 1400 50  0001 C CNN
F 3 "https://product.tdk.com/info/en/catalog/datasheets/mlcc_automotive_soft_en.pdf?ref_disty=digikey" H 4050 1550 50  0001 C CNN
F 4 "CGA5L1X7R1E106K160AE" H 4050 1550 50  0001 C CNN "Part Number"
	1    4050 1550
	1    0    0    -1  
$EndComp
$Comp
L Device:C C6
U 1 1 60A91F0C
P 4450 1550
F 0 "C6" H 4565 1596 50  0000 L CNN
F 1 "100n" H 4565 1505 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 4488 1400 50  0001 C CNN
F 3 "http://www.samsungsem.com/kr/support/product-search/mlcc/__icsFiles/afieldfile/2019/08/26/CL10B104KB8WPND-19.pdf" H 4450 1550 50  0001 C CNN
F 4 "CL10B104KB8WPND" H 4450 1550 50  0001 C CNN "Part Number"
	1    4450 1550
	1    0    0    -1  
$EndComp
$Comp
L Device:R R3
U 1 1 60A92485
P 3750 1550
F 0 "R3" H 3820 1596 50  0000 L CNN
F 1 "576k" H 3820 1505 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" V 3680 1550 50  0001 C CNN
F 3 "https://www.seielect.com/catalog/sei-rncf.pdf" H 3750 1550 50  0001 C CNN
F 4 "RNCF0603DTC576K" H 3750 1550 50  0001 C CNN "Part Number"
	1    3750 1550
	1    0    0    -1  
$EndComp
$Comp
L Device:R R4
U 1 1 60A92950
P 3750 1950
F 0 "R4" H 3820 1996 50  0000 L CNN
F 1 "49k9" H 3820 1905 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" V 3680 1950 50  0001 C CNN
F 3 "https://industrial.panasonic.com/cdbs/www-data/pdf/RDA0000/AOA0000C304.pdf" H 3750 1950 50  0001 C CNN
F 4 "ERJ-3EKF4992V" H 3750 1950 50  0001 C CNN "Part Number"
	1    3750 1950
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR09
U 1 1 60A92FC8
P 3750 2200
F 0 "#PWR09" H 3750 1950 50  0001 C CNN
F 1 "GND" H 3755 2027 50  0000 C CNN
F 2 "" H 3750 2200 50  0001 C CNN
F 3 "" H 3750 2200 50  0001 C CNN
	1    3750 2200
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR010
U 1 1 60A93502
P 4050 1800
F 0 "#PWR010" H 4050 1550 50  0001 C CNN
F 1 "GND" H 4055 1627 50  0000 C CNN
F 2 "" H 4050 1800 50  0001 C CNN
F 3 "" H 4050 1800 50  0001 C CNN
	1    4050 1800
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR014
U 1 1 60A93787
P 4450 1800
F 0 "#PWR014" H 4450 1550 50  0001 C CNN
F 1 "GND" H 4455 1627 50  0000 C CNN
F 2 "" H 4450 1800 50  0001 C CNN
F 3 "" H 4450 1800 50  0001 C CNN
	1    4450 1800
	1    0    0    -1  
$EndComp
Wire Wire Line
	4050 1700 4050 1800
Wire Wire Line
	4450 1700 4450 1800
Wire Wire Line
	3750 1700 3750 1750
Wire Wire Line
	3750 2100 3750 2200
Connection ~ 3750 1400
Wire Wire Line
	3750 1400 4050 1400
Connection ~ 4050 1400
Wire Wire Line
	3300 1400 3050 1400
Wire Wire Line
	3600 1400 3750 1400
Wire Wire Line
	3750 1750 3650 1750
Wire Wire Line
	3650 1750 3650 1700
Wire Wire Line
	3650 1700 3050 1700
Connection ~ 3750 1750
Wire Wire Line
	3750 1750 3750 1800
$Comp
L Device:C C2
U 1 1 60A945AF
P 1450 1500
F 0 "C2" H 1565 1546 50  0000 L CNN
F 1 "1u" H 1565 1455 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 1488 1350 50  0001 C CNN
F 3 "https://product.tdk.com/info/en/catalog/datasheets/mlcc_commercial_midvoltage_en.pdf?ref_disty=digikey" H 1450 1500 50  0001 C CNN
F 4 "C3216X7R2A105K160AA" H 1450 1500 50  0001 C CNN "Part Number"
	1    1450 1500
	1    0    0    -1  
$EndComp
$Comp
L Device:C C1
U 1 1 60A94BE6
P 1150 1500
F 0 "C1" H 1265 1546 50  0000 L CNN
F 1 "10n" H 1265 1455 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 1188 1350 50  0001 C CNN
F 3 "https://datasheets.avx.com/X7RDielectric.pdf" H 1150 1500 50  0001 C CNN
F 4 "06035C103JAT2A" H 1150 1500 50  0001 C CNN "Part Number"
	1    1150 1500
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR03
U 1 1 60A94EDC
P 1450 1650
F 0 "#PWR03" H 1450 1400 50  0001 C CNN
F 1 "GND" H 1455 1477 50  0000 C CNN
F 2 "" H 1450 1650 50  0001 C CNN
F 3 "" H 1450 1650 50  0001 C CNN
	1    1450 1650
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR02
U 1 1 60A951CE
P 1150 1650
F 0 "#PWR02" H 1150 1400 50  0001 C CNN
F 1 "GND" H 1155 1477 50  0000 C CNN
F 2 "" H 1150 1650 50  0001 C CNN
F 3 "" H 1150 1650 50  0001 C CNN
	1    1150 1650
	1    0    0    -1  
$EndComp
Wire Wire Line
	1150 1350 1450 1350
Connection ~ 1450 1350
Wire Wire Line
	1450 1350 1900 1350
Wire Wire Line
	2050 1450 1900 1450
Wire Wire Line
	1900 1450 1900 1350
Connection ~ 1900 1350
Wire Wire Line
	1900 1350 2050 1350
$Comp
L Regulator_Linear:TPS76350 U3
U 1 1 60A95CE9
P 5650 1500
F 0 "U3" H 5650 1842 50  0000 C CNN
F 1 "TPS76350" H 5650 1751 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23-5" H 5650 1825 50  0001 C CIN
F 3 "http://www.ti.com/lit/ds/symlink/tps763.pdf" H 5650 1500 50  0001 C CNN
F 4 "TPS76350-Q1" H 5650 1500 50  0001 C CNN "Part Number"
	1    5650 1500
	1    0    0    -1  
$EndComp
Wire Wire Line
	4450 1400 4950 1400
Connection ~ 4450 1400
Wire Wire Line
	5350 1500 5200 1500
Wire Wire Line
	5200 1500 5200 1400
Connection ~ 5200 1400
Wire Wire Line
	5200 1400 5350 1400
$Comp
L Device:C C7
U 1 1 60A972D2
P 4950 1550
F 0 "C7" H 5065 1596 50  0000 L CNN
F 1 "1u" H 5065 1505 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 4988 1400 50  0001 C CNN
F 3 "http://www.samsungsem.com/kr/support/product-search/mlcc/CL21B105KOFVPNE.jsp" H 4950 1550 50  0001 C CNN
F 4 "CL21B105KOFVPNE" H 4950 1550 50  0001 C CNN "Part Number"
	1    4950 1550
	1    0    0    -1  
$EndComp
Connection ~ 4950 1400
Wire Wire Line
	4950 1400 5200 1400
$Comp
L power:GND #PWR015
U 1 1 60A97798
P 4950 1700
F 0 "#PWR015" H 4950 1450 50  0001 C CNN
F 1 "GND" H 4955 1527 50  0000 C CNN
F 2 "" H 4950 1700 50  0001 C CNN
F 3 "" H 4950 1700 50  0001 C CNN
	1    4950 1700
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR018
U 1 1 60A97B3A
P 5650 1800
F 0 "#PWR018" H 5650 1550 50  0001 C CNN
F 1 "GND" H 5655 1627 50  0000 C CNN
F 2 "" H 5650 1800 50  0001 C CNN
F 3 "" H 5650 1800 50  0001 C CNN
	1    5650 1800
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR019
U 1 1 60A98235
P 6050 1700
F 0 "#PWR019" H 6050 1450 50  0001 C CNN
F 1 "GND" H 6055 1527 50  0000 C CNN
F 2 "" H 6050 1700 50  0001 C CNN
F 3 "" H 6050 1700 50  0001 C CNN
	1    6050 1700
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR021
U 1 1 60A98E87
P 6400 1400
F 0 "#PWR021" H 6400 1250 50  0001 C CNN
F 1 "+5V" H 6415 1573 50  0000 C CNN
F 2 "" H 6400 1400 50  0001 C CNN
F 3 "" H 6400 1400 50  0001 C CNN
	1    6400 1400
	1    0    0    -1  
$EndComp
Wire Wire Line
	3100 1800 3050 1800
$Comp
L MCU_Microchip_ATtiny:ATtiny85-20SU U2
U 1 1 60AA6F3D
P 4100 4400
F 0 "U2" H 3571 4446 50  0000 R CNN
F 1 "ATtiny85-20SU" H 3571 4355 50  0000 R CNN
F 2 "Package_SO:SOIJ-8_5.3x5.3mm_P1.27mm" H 4100 4400 50  0001 C CIN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/atmel-2586-avr-8-bit-microcontroller-attiny25-attiny45-attiny85_datasheet.pdf" H 4100 4400 50  0001 C CNN
F 4 "ATtiny85-20SU" H 4100 4400 50  0001 C CNN "Part Number"
	1    4100 4400
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR012
U 1 1 60AA760B
P 4100 5000
F 0 "#PWR012" H 4100 4750 50  0001 C CNN
F 1 "GND" H 4105 4827 50  0000 C CNN
F 2 "" H 4100 5000 50  0001 C CNN
F 3 "" H 4100 5000 50  0001 C CNN
	1    4100 5000
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR011
U 1 1 60AA7C64
P 4100 3250
F 0 "#PWR011" H 4100 3100 50  0001 C CNN
F 1 "+5V" H 4115 3423 50  0000 C CNN
F 2 "" H 4100 3250 50  0001 C CNN
F 3 "" H 4100 3250 50  0001 C CNN
	1    4100 3250
	1    0    0    -1  
$EndComp
$Comp
L Device:C C4
U 1 1 60AA8306
P 3650 3500
F 0 "C4" H 3765 3546 50  0000 L CNN
F 1 "100n" H 3765 3455 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 3688 3350 50  0001 C CNN
F 3 "http://www.samsungsem.com/kr/support/product-search/mlcc/__icsFiles/afieldfile/2019/08/26/CL10B104KB8WPND-19.pdf" H 3650 3500 50  0001 C CNN
F 4 "CL10B104KB8WPND" H 3650 3500 50  0001 C CNN "Part Number"
	1    3650 3500
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR08
U 1 1 60AA86AE
P 3650 3650
F 0 "#PWR08" H 3650 3400 50  0001 C CNN
F 1 "GND" H 3655 3477 50  0000 C CNN
F 2 "" H 3650 3650 50  0001 C CNN
F 3 "" H 3650 3650 50  0001 C CNN
	1    3650 3650
	1    0    0    -1  
$EndComp
Wire Wire Line
	4100 3800 4100 3350
Wire Wire Line
	4100 3350 3650 3350
Connection ~ 4100 3350
Wire Wire Line
	4100 3350 4100 3250
$Comp
L Device:C C13
U 1 1 60AAA496
P 7550 5500
F 0 "C13" H 7665 5546 50  0000 L CNN
F 1 "1u" H 7665 5455 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 7588 5350 50  0001 C CNN
F 3 "http://www.samsungsem.com/kr/support/product-search/mlcc/CL21B105KOFVPNE.jsp" H 7550 5500 50  0001 C CNN
F 4 "CL21B105KOFVPNE" H 7550 5500 50  0001 C CNN "Part Number"
	1    7550 5500
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR026
U 1 1 60AAAB54
P 7550 5650
F 0 "#PWR026" H 7550 5400 50  0001 C CNN
F 1 "GND" H 7555 5477 50  0000 C CNN
F 2 "" H 7550 5650 50  0001 C CNN
F 3 "" H 7550 5650 50  0001 C CNN
	1    7550 5650
	1    0    0    -1  
$EndComp
Wire Wire Line
	7550 5250 7300 5250
Text GLabel 8100 5250 2    50   Output ~ 0
PWM_OUT
Wire Wire Line
	8100 5250 7550 5250
Connection ~ 7550 5250
Wire Wire Line
	7550 5250 7550 5350
$Comp
L Device:C C12
U 1 1 60AB1B72
P 7100 4550
F 0 "C12" H 7215 4596 50  0000 L CNN
F 1 "10n" H 7215 4505 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 7138 4400 50  0001 C CNN
F 3 "https://datasheets.avx.com/X7RDielectric.pdf" H 7100 4550 50  0001 C CNN
F 4 "06035C103JAT2A" H 7100 4550 50  0001 C CNN "Part Number"
	1    7100 4550
	-1   0    0    -1  
$EndComp
$Comp
L power:GND #PWR025
U 1 1 60AB1B78
P 7100 4700
F 0 "#PWR025" H 7100 4450 50  0001 C CNN
F 1 "GND" H 7105 4527 50  0000 C CNN
F 2 "" H 7100 4700 50  0001 C CNN
F 3 "" H 7100 4700 50  0001 C CNN
	1    7100 4700
	-1   0    0    -1  
$EndComp
Wire Wire Line
	7100 4300 7350 4300
Wire Wire Line
	7650 4300 8200 4300
Wire Wire Line
	7100 4300 7100 4400
Text GLabel 8350 4300 2    50   Input ~ 0
TORQUE_IN
$Comp
L Device:C C11
U 1 1 60AB31C7
P 7100 3800
F 0 "C11" H 7215 3846 50  0000 L CNN
F 1 "10n" H 7215 3755 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 7138 3650 50  0001 C CNN
F 3 "https://datasheets.avx.com/X7RDielectric.pdf" H 7100 3800 50  0001 C CNN
F 4 "06035C103JAT2A" H 7100 3800 50  0001 C CNN "Part Number"
	1    7100 3800
	-1   0    0    -1  
$EndComp
$Comp
L power:GND #PWR024
U 1 1 60AB31CD
P 7100 3950
F 0 "#PWR024" H 7100 3700 50  0001 C CNN
F 1 "GND" H 7105 3777 50  0000 C CNN
F 2 "" H 7100 3950 50  0001 C CNN
F 3 "" H 7100 3950 50  0001 C CNN
	1    7100 3950
	-1   0    0    -1  
$EndComp
Wire Wire Line
	7100 3550 7350 3550
Wire Wire Line
	7650 3550 8200 3550
Connection ~ 7100 3550
Wire Wire Line
	7100 3550 7100 3650
Text GLabel 8350 3550 2    50   Input ~ 0
CADENCE_IN
$Comp
L Device:R R7
U 1 1 60AB447D
P 7500 2800
F 0 "R7" V 7293 2800 50  0000 C CNN
F 1 "10k" V 7384 2800 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" V 7430 2800 50  0001 C CNN
F 3 "https://www.te.com/commerce/DocumentDelivery/DDEController?Action=srchrtrv&DocNm=1773204-3&DocType=DS&DocLang=English" H 7500 2800 50  0001 C CNN
F 4 "CRGCQ0603F10K" H 7500 2800 50  0001 C CNN "Part Number"
	1    7500 2800
	0    -1   1    0   
$EndComp
$Comp
L Device:C C10
U 1 1 60AB4483
P 7100 3050
F 0 "C10" H 7215 3096 50  0000 L CNN
F 1 "10n" H 7215 3005 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 7138 2900 50  0001 C CNN
F 3 "https://datasheets.avx.com/X7RDielectric.pdf" H 7100 3050 50  0001 C CNN
F 4 "06035C103JAT2A" H 7100 3050 50  0001 C CNN "Part Number"
	1    7100 3050
	-1   0    0    -1  
$EndComp
$Comp
L power:GND #PWR023
U 1 1 60AB4489
P 7100 3200
F 0 "#PWR023" H 7100 2950 50  0001 C CNN
F 1 "GND" H 7105 3027 50  0000 C CNN
F 2 "" H 7100 3200 50  0001 C CNN
F 3 "" H 7100 3200 50  0001 C CNN
	1    7100 3200
	-1   0    0    -1  
$EndComp
Wire Wire Line
	7100 2800 7350 2800
Wire Wire Line
	7650 2800 7850 2800
Connection ~ 7100 2800
Wire Wire Line
	7100 2800 7100 2900
Text GLabel 8350 2800 2    50   Input ~ 0
BUTTON_IN
$Comp
L Device:R R1
U 1 1 60AB82B3
P 1750 2100
F 0 "R1" H 1820 2146 50  0000 L CNN
F 1 "69.8k" H 1820 2055 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" V 1680 2100 50  0001 C CNN
F 3 "https://www.seielect.com/catalog/sei-rmcf_rmcp.pdf" H 1750 2100 50  0001 C CNN
F 4 "RMCF0603FT69K8" H 1750 2100 50  0001 C CNN "Part Number"
	1    1750 2100
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR04
U 1 1 60AB8615
P 1750 2250
F 0 "#PWR04" H 1750 2000 50  0001 C CNN
F 1 "GND" H 1755 2077 50  0000 C CNN
F 2 "" H 1750 2250 50  0001 C CNN
F 3 "" H 1750 2250 50  0001 C CNN
	1    1750 2250
	1    0    0    -1  
$EndComp
Wire Wire Line
	1750 1950 1750 1550
Wire Wire Line
	1750 1550 2050 1550
$Comp
L Device:CP C8
U 1 1 60ABA134
P 6050 1550
F 0 "C8" H 6168 1596 50  0000 L CNN
F 1 "4u7" H 6168 1505 50  0000 L CNN
F 2 "Capacitor_SMD:CP_Elec_4x5.4" H 6088 1400 50  0001 C CNN
F 3 "https://industrial.panasonic.com/cdbs/www-data/pdf/RDE0000/ABA0000C1151.pdf" H 6050 1550 50  0001 C CNN
F 4 "EEE-HA1E4R7R" H 6050 1550 50  0001 C CNN "Part Number"
	1    6050 1550
	1    0    0    -1  
$EndComp
Wire Wire Line
	5950 1400 6050 1400
Connection ~ 6050 1400
Wire Wire Line
	6050 1400 6350 1400
$Comp
L Device:C C9
U 1 1 60ABAE24
P 6350 1550
F 0 "C9" H 6465 1596 50  0000 L CNN
F 1 "100n" H 6465 1505 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 6388 1400 50  0001 C CNN
F 3 "http://www.samsungsem.com/kr/support/product-search/mlcc/__icsFiles/afieldfile/2019/08/26/CL10B104KB8WPND-19.pdf" H 6350 1550 50  0001 C CNN
F 4 "CL10B104KB8WPND" H 6350 1550 50  0001 C CNN "Part Number"
	1    6350 1550
	1    0    0    -1  
$EndComp
Connection ~ 6350 1400
Wire Wire Line
	6350 1400 6400 1400
$Comp
L power:GND #PWR020
U 1 1 60ABB122
P 6350 1700
F 0 "#PWR020" H 6350 1450 50  0001 C CNN
F 1 "GND" H 6355 1527 50  0000 C CNN
F 2 "" H 6350 1700 50  0001 C CNN
F 3 "" H 6350 1700 50  0001 C CNN
	1    6350 1700
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR05
U 1 1 60ABB2FC
P 2600 2000
F 0 "#PWR05" H 2600 1750 50  0001 C CNN
F 1 "GND" H 2605 1827 50  0000 C CNN
F 2 "" H 2600 2000 50  0001 C CNN
F 3 "" H 2600 2000 50  0001 C CNN
	1    2600 2000
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x04 J2
U 1 1 60ABC766
P 8850 1400
F 0 "J2" H 8930 1392 50  0000 L CNN
F 1 "Conn_01x04" H 8930 1301 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical" H 8850 1400 50  0001 C CNN
F 3 "~" H 8850 1400 50  0001 C CNN
	1    8850 1400
	1    0    0    -1  
$EndComp
$Comp
L Device:R R10
U 1 1 60AC07B5
P 7850 2650
F 0 "R10" H 7780 2696 50  0000 R CNN
F 1 "10k" H 7780 2605 50  0000 R CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" V 7780 2650 50  0001 C CNN
F 3 "https://www.te.com/commerce/DocumentDelivery/DDEController?Action=srchrtrv&DocNm=1773204-3&DocType=DS&DocLang=English" H 7850 2650 50  0001 C CNN
F 4 "CRGCQ0603F10K" H 7850 2650 50  0001 C CNN "Part Number"
	1    7850 2650
	-1   0    0    -1  
$EndComp
Connection ~ 7850 2800
Wire Wire Line
	7850 2800 8200 2800
$Comp
L power:+5V #PWR029
U 1 1 60AC0C34
P 7850 2500
F 0 "#PWR029" H 7850 2350 50  0001 C CNN
F 1 "+5V" H 7865 2673 50  0000 C CNN
F 2 "" H 7850 2500 50  0001 C CNN
F 3 "" H 7850 2500 50  0001 C CNN
	1    7850 2500
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR022
U 1 1 60AC1FF1
P 6600 5600
F 0 "#PWR022" H 6600 5350 50  0001 C CNN
F 1 "GND" H 6605 5427 50  0000 C CNN
F 2 "" H 6600 5600 50  0001 C CNN
F 3 "" H 6600 5600 50  0001 C CNN
	1    6600 5600
	1    0    0    -1  
$EndComp
Wire Wire Line
	6600 5300 6600 5250
Connection ~ 6600 5250
Wire Wire Line
	6600 5250 7000 5250
$Comp
L Device:R R8
U 1 1 60AC311E
P 7500 3550
F 0 "R8" V 7293 3550 50  0000 C CNN
F 1 "10k" V 7384 3550 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" V 7430 3550 50  0001 C CNN
F 3 "https://www.te.com/commerce/DocumentDelivery/DDEController?Action=srchrtrv&DocNm=1773204-3&DocType=DS&DocLang=English" H 7500 3550 50  0001 C CNN
F 4 "CRGCQ0603F10K" H 7500 3550 50  0001 C CNN "Part Number"
	1    7500 3550
	0    -1   1    0   
$EndComp
$Comp
L Device:R R9
U 1 1 60AC3559
P 7500 4300
F 0 "R9" V 7293 4300 50  0000 C CNN
F 1 "10k" V 7384 4300 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" V 7430 4300 50  0001 C CNN
F 3 "https://www.te.com/commerce/DocumentDelivery/DDEController?Action=srchrtrv&DocNm=1773204-3&DocType=DS&DocLang=English" H 7500 4300 50  0001 C CNN
F 4 "CRGCQ0603F10K" H 7500 4300 50  0001 C CNN "Part Number"
	1    7500 4300
	0    -1   1    0   
$EndComp
$Comp
L Device:R R6
U 1 1 60AC391B
P 7150 5250
F 0 "R6" V 6943 5250 50  0000 C CNN
F 1 "10k" V 7034 5250 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" V 7080 5250 50  0001 C CNN
F 3 "https://www.te.com/commerce/DocumentDelivery/DDEController?Action=srchrtrv&DocNm=1773204-3&DocType=DS&DocLang=English" H 7150 5250 50  0001 C CNN
F 4 "CRGCQ0603F10K" H 7150 5250 50  0001 C CNN "Part Number"
	1    7150 5250
	0    -1   1    0   
$EndComp
$Comp
L Device:R R5
U 1 1 60AC3CB4
P 6600 5450
F 0 "R5" H 6530 5496 50  0000 R CNN
F 1 "10k" H 6530 5405 50  0000 R CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" V 6530 5450 50  0001 C CNN
F 3 "https://www.te.com/commerce/DocumentDelivery/DDEController?Action=srchrtrv&DocNm=1773204-3&DocType=DS&DocLang=English" H 6600 5450 50  0001 C CNN
F 4 "CRGCQ0603F10K" H 6600 5450 50  0001 C CNN "Part Number"
	1    6600 5450
	-1   0    0    -1  
$EndComp
$Comp
L Device:C C17
U 1 1 60AC45B7
P 8200 4450
F 0 "C17" H 8315 4496 50  0000 L CNN
F 1 "10n" H 8315 4405 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 8238 4300 50  0001 C CNN
F 3 "https://datasheets.avx.com/X7RDielectric.pdf" H 8200 4450 50  0001 C CNN
F 4 "06035C103JAT2A" H 8200 4450 50  0001 C CNN "Part Number"
	1    8200 4450
	1    0    0    -1  
$EndComp
Connection ~ 8200 4300
Wire Wire Line
	8200 4300 8350 4300
$Comp
L power:GND #PWR032
U 1 1 60AC52A7
P 8200 4600
F 0 "#PWR032" H 8200 4350 50  0001 C CNN
F 1 "GND" H 8205 4427 50  0000 C CNN
F 2 "" H 8200 4600 50  0001 C CNN
F 3 "" H 8200 4600 50  0001 C CNN
	1    8200 4600
	1    0    0    -1  
$EndComp
$Comp
L Device:C C16
U 1 1 60AC5C55
P 8200 3800
F 0 "C16" H 8315 3846 50  0000 L CNN
F 1 "10n" H 8315 3755 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 8238 3650 50  0001 C CNN
F 3 "https://datasheets.avx.com/X7RDielectric.pdf" H 8200 3800 50  0001 C CNN
F 4 "06035C103JAT2A" H 8200 3800 50  0001 C CNN "Part Number"
	1    8200 3800
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR031
U 1 1 60AC5C5B
P 8200 3950
F 0 "#PWR031" H 8200 3700 50  0001 C CNN
F 1 "GND" H 8205 3777 50  0000 C CNN
F 2 "" H 8200 3950 50  0001 C CNN
F 3 "" H 8200 3950 50  0001 C CNN
	1    8200 3950
	1    0    0    -1  
$EndComp
$Comp
L Device:C C15
U 1 1 60AC6B0A
P 8200 3050
F 0 "C15" H 8315 3096 50  0000 L CNN
F 1 "10n" H 8315 3005 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 8238 2900 50  0001 C CNN
F 3 "https://datasheets.avx.com/X7RDielectric.pdf" H 8200 3050 50  0001 C CNN
F 4 "06035C103JAT2A" H 8200 3050 50  0001 C CNN "Part Number"
	1    8200 3050
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR030
U 1 1 60AC6B10
P 8200 3200
F 0 "#PWR030" H 8200 2950 50  0001 C CNN
F 1 "GND" H 8205 3027 50  0000 C CNN
F 2 "" H 8200 3200 50  0001 C CNN
F 3 "" H 8200 3200 50  0001 C CNN
	1    8200 3200
	1    0    0    -1  
$EndComp
Text GLabel 8650 1500 0    50   Output ~ 0
CADENCE_IN
Text GLabel 8650 1600 0    50   Output ~ 0
TORQUE_IN
$Comp
L power:GND #PWR028
U 1 1 60AC82D0
P 7800 1450
F 0 "#PWR028" H 7800 1200 50  0001 C CNN
F 1 "GND" H 7805 1277 50  0000 C CNN
F 2 "" H 7800 1450 50  0001 C CNN
F 3 "" H 7800 1450 50  0001 C CNN
	1    7800 1450
	1    0    0    -1  
$EndComp
Wire Wire Line
	8650 1400 7800 1400
Wire Wire Line
	7800 1400 7800 1450
Wire Wire Line
	8650 1300 8350 1300
$Comp
L Device:C C14
U 1 1 60ACABFB
P 7800 1250
F 0 "C14" H 7915 1296 50  0000 L CNN
F 1 "1u" H 7915 1205 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 7838 1100 50  0001 C CNN
F 3 "http://www.samsungsem.com/kr/support/product-search/mlcc/CL21B105KOFVPNE.jsp" H 7800 1250 50  0001 C CNN
F 4 "CL21B105KOFVPNE" H 7800 1250 50  0001 C CNN "Part Number"
	1    7800 1250
	1    0    0    -1  
$EndComp
Connection ~ 7800 1400
Wire Wire Line
	7800 1100 8350 1100
Wire Wire Line
	8350 1100 8350 1300
Wire Wire Line
	5500 5250 5500 4200
Wire Wire Line
	5500 4200 5200 4200
Wire Wire Line
	5500 5250 6600 5250
Wire Wire Line
	5850 3550 5850 4300
Wire Wire Line
	5850 4300 5100 4300
Wire Wire Line
	5850 3550 7100 3550
Wire Wire Line
	5500 2800 5500 4100
Wire Wire Line
	5500 4100 5300 4100
Wire Wire Line
	5500 2800 7100 2800
Wire Wire Line
	7100 4300 6350 4300
Wire Wire Line
	6350 4300 6350 4400
Wire Wire Line
	6350 4400 4700 4400
Connection ~ 7100 4300
Connection ~ 5300 4100
Wire Wire Line
	5300 4100 4700 4100
Connection ~ 5200 4200
Wire Wire Line
	5200 4200 4700 4200
Connection ~ 5100 4300
Wire Wire Line
	5100 4300 4700 4300
Wire Wire Line
	4700 4600 5000 4600
$Comp
L power:+5V #PWR017
U 1 1 60ADF8F7
P 4150 5650
F 0 "#PWR017" H 4150 5500 50  0001 C CNN
F 1 "+5V" H 4165 5823 50  0000 C CNN
F 2 "" H 4150 5650 50  0001 C CNN
F 3 "" H 4150 5650 50  0001 C CNN
	1    4150 5650
	1    0    0    -1  
$EndComp
Text GLabel 9050 2200 0    50   Input ~ 0
PWM_OUT
$Comp
L power:GND #PWR034
U 1 1 60AE24AB
P 8500 2100
F 0 "#PWR034" H 8500 1850 50  0001 C CNN
F 1 "GND" H 8505 1927 50  0000 C CNN
F 2 "" H 8500 2100 50  0001 C CNN
F 3 "" H 8500 2100 50  0001 C CNN
	1    8500 2100
	1    0    0    -1  
$EndComp
Wire Wire Line
	8500 2100 9050 2100
$Comp
L power:VBUS #PWR01
U 1 1 60AE4986
P 1150 1350
F 0 "#PWR01" H 1150 1200 50  0001 C CNN
F 1 "VBUS" H 1165 1523 50  0000 C CNN
F 2 "" H 1150 1350 50  0001 C CNN
F 3 "" H 1150 1350 50  0001 C CNN
	1    1150 1350
	1    0    0    -1  
$EndComp
Connection ~ 1150 1350
Wire Wire Line
	4050 1400 4300 1400
$Comp
L power:+10V #PWR0101
U 1 1 60AEEFE0
P 4300 1400
F 0 "#PWR0101" H 4300 1250 50  0001 C CNN
F 1 "+10V" H 4315 1573 50  0000 C CNN
F 2 "" H 4300 1400 50  0001 C CNN
F 3 "" H 4300 1400 50  0001 C CNN
	1    4300 1400
	1    0    0    -1  
$EndComp
Connection ~ 4300 1400
Wire Wire Line
	4300 1400 4450 1400
$Comp
L power:+10V #PWR0102
U 1 1 60AEFB67
P 7800 1100
F 0 "#PWR0102" H 7800 950 50  0001 C CNN
F 1 "+10V" H 7815 1273 50  0000 C CNN
F 2 "" H 7800 1100 50  0001 C CNN
F 3 "" H 7800 1100 50  0001 C CNN
	1    7800 1100
	1    0    0    -1  
$EndComp
Connection ~ 7800 1100
NoConn ~ 2050 1650
NoConn ~ 2050 1750
NoConn ~ 2050 1850
$Comp
L power:GND #PWR0103
U 1 1 60B290A0
P 3100 1800
F 0 "#PWR0103" H 3100 1550 50  0001 C CNN
F 1 "GND" H 3105 1627 50  0000 C CNN
F 2 "" H 3100 1800 50  0001 C CNN
F 3 "" H 3100 1800 50  0001 C CNN
	1    3100 1800
	1    0    0    -1  
$EndComp
$Comp
L Connector:AVR-ISP-6 J1
U 1 1 60B44FFA
P 4250 6150
F 0 "J1" H 3921 6246 50  0000 R CNN
F 1 "AVR-ISP-6" H 3921 6155 50  0000 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_2x03_P2.54mm_Vertical" V 4000 6200 50  0001 C CNN
F 3 " ~" H 2975 5600 50  0001 C CNN
	1    4250 6150
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0104
U 1 1 60B45C60
P 4150 6550
F 0 "#PWR0104" H 4150 6300 50  0001 C CNN
F 1 "GND" H 4155 6377 50  0000 C CNN
F 2 "" H 4150 6550 50  0001 C CNN
F 3 "" H 4150 6550 50  0001 C CNN
	1    4150 6550
	1    0    0    -1  
$EndComp
Wire Wire Line
	5000 6250 4650 6250
Wire Wire Line
	5000 4600 5000 6250
Wire Wire Line
	4650 6150 5100 6150
Wire Wire Line
	5100 4300 5100 6150
Wire Wire Line
	4650 5950 5200 5950
Wire Wire Line
	5200 4200 5200 5950
Wire Wire Line
	4650 6050 5300 6050
Wire Wire Line
	5300 4100 5300 6050
Wire Wire Line
	8200 3650 8200 3550
Connection ~ 8200 3550
Wire Wire Line
	8200 3550 8350 3550
Wire Wire Line
	8200 2900 8200 2800
Connection ~ 8200 2800
Wire Wire Line
	8200 2800 8350 2800
$Comp
L Connector_Generic:Conn_01x02 J4
U 1 1 60B78B61
P 9700 2800
F 0 "J4" H 9780 2792 50  0000 L CNN
F 1 "Conn_01x02" H 9780 2701 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 9700 2800 50  0001 C CNN
F 3 "~" H 9700 2800 50  0001 C CNN
	1    9700 2800
	1    0    0    -1  
$EndComp
Text GLabel 9500 2800 0    50   Output ~ 0
BUTTON_IN
$Comp
L power:GND #PWR06
U 1 1 60B79B1A
P 9500 2900
F 0 "#PWR06" H 9500 2650 50  0001 C CNN
F 1 "GND" H 9505 2727 50  0000 C CNN
F 2 "" H 9500 2900 50  0001 C CNN
F 3 "" H 9500 2900 50  0001 C CNN
	1    9500 2900
	1    0    0    -1  
$EndComp
$Comp
L Device:R R2
U 1 1 60C7B287
P 1050 4300
F 0 "R2" V 843 4300 50  0000 C CNN
F 1 "R0005" V 934 4300 50  0000 C CNN
F 2 "Resistor_SMD:R_1206_3216Metric" V 980 4300 50  0001 C CNN
F 3 "https://www.seielect.com/catalog/sei-hcs.pdf" H 1050 4300 50  0001 C CNN
F 4 "HCS1206FTL500" H 1050 4300 50  0001 C CNN "Part Number"
	1    1050 4300
	0    1    1    0   
$EndComp
Wire Wire Line
	1500 5450 800  5450
Wire Wire Line
	800  5450 800  4300
Wire Wire Line
	800  4300 900  4300
Wire Wire Line
	1500 5650 1400 5650
Wire Wire Line
	1400 5650 1400 4300
Wire Wire Line
	1400 4300 1200 4300
$Comp
L power:GND #PWR013
U 1 1 60C85154
P 1700 5850
F 0 "#PWR013" H 1700 5600 50  0001 C CNN
F 1 "GND" H 1705 5677 50  0000 C CNN
F 2 "" H 1700 5850 50  0001 C CNN
F 3 "" H 1700 5850 50  0001 C CNN
	1    1700 5850
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR07
U 1 1 60C855B0
P 1700 4850
F 0 "#PWR07" H 1700 4700 50  0001 C CNN
F 1 "+5V" H 1715 5023 50  0000 C CNN
F 2 "" H 1700 4850 50  0001 C CNN
F 3 "" H 1700 4850 50  0001 C CNN
	1    1700 4850
	1    0    0    -1  
$EndComp
$Comp
L Device:C C3
U 1 1 60C85B23
P 2050 5100
F 0 "C3" H 2165 5146 50  0000 L CNN
F 1 "100n" H 2165 5055 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 2088 4950 50  0001 C CNN
F 3 "http://www.samsungsem.com/kr/support/product-search/mlcc/__icsFiles/afieldfile/2019/08/26/CL10B104KB8WPND-19.pdf" H 2050 5100 50  0001 C CNN
F 4 "CL10B104KB8WPND" H 2050 5100 50  0001 C CNN "Part Number"
	1    2050 5100
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR016
U 1 1 60C865C3
P 2050 5250
F 0 "#PWR016" H 2050 5000 50  0001 C CNN
F 1 "GND" H 2055 5077 50  0000 C CNN
F 2 "" H 2050 5250 50  0001 C CNN
F 3 "" H 2050 5250 50  0001 C CNN
	1    2050 5250
	1    0    0    -1  
$EndComp
Wire Wire Line
	1700 5250 1700 4900
Wire Wire Line
	2050 4950 2050 4900
Wire Wire Line
	2050 4900 1700 4900
Connection ~ 1700 4900
Wire Wire Line
	1700 4900 1700 4850
$Comp
L Device:R R11
U 1 1 60C8C238
P 2550 5550
F 0 "R11" V 2343 5550 50  0000 C CNN
F 1 "10k" V 2434 5550 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad0.98x0.95mm_HandSolder" V 2480 5550 50  0001 C CNN
F 3 "https://www.te.com/commerce/DocumentDelivery/DDEController?Action=srchrtrv&DocNm=1773204-3&DocType=DS&DocLang=English" H 2550 5550 50  0001 C CNN
F 4 "CRGCQ0603F10K" H 2550 5550 50  0001 C CNN "Part Number"
	1    2550 5550
	0    -1   1    0   
$EndComp
$Comp
L Device:C C18
U 1 1 60C8DA74
P 2900 5800
F 0 "C18" H 3015 5846 50  0000 L CNN
F 1 "10n" H 3015 5755 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 2938 5650 50  0001 C CNN
F 3 "https://datasheets.avx.com/X7RDielectric.pdf" H 2900 5800 50  0001 C CNN
F 4 "06035C103JAT2A" H 2900 5800 50  0001 C CNN "Part Number"
	1    2900 5800
	-1   0    0    -1  
$EndComp
$Comp
L power:GND #PWR027
U 1 1 60C8DA7A
P 2900 5950
F 0 "#PWR027" H 2900 5700 50  0001 C CNN
F 1 "GND" H 2905 5777 50  0000 C CNN
F 2 "" H 2900 5950 50  0001 C CNN
F 3 "" H 2900 5950 50  0001 C CNN
	1    2900 5950
	-1   0    0    -1  
$EndComp
Wire Wire Line
	2900 5650 2900 5550
Wire Wire Line
	2900 5550 2700 5550
Wire Wire Line
	2400 5550 2100 5550
Wire Wire Line
	2900 5550 3450 5550
Wire Wire Line
	3450 5550 3450 5300
Wire Wire Line
	3450 5300 4900 5300
Wire Wire Line
	4900 5300 4900 4500
Wire Wire Line
	4900 4500 4700 4500
Connection ~ 2900 5550
$Comp
L Connector_Generic:Conn_01x02 J3
U 1 1 60CD27FA
P 9250 2100
F 0 "J3" H 9330 2092 50  0000 L CNN
F 1 "Conn_01x02" H 9330 2001 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 9250 2100 50  0001 C CNN
F 3 "~" H 9250 2100 50  0001 C CNN
	1    9250 2100
	1    0    0    -1  
$EndComp
$Comp
L power:VBUS #PWR0105
U 1 1 60CD3286
P 800 4300
F 0 "#PWR0105" H 800 4150 50  0001 C CNN
F 1 "VBUS" H 815 4473 50  0000 C CNN
F 2 "" H 800 4300 50  0001 C CNN
F 3 "" H 800 4300 50  0001 C CNN
	1    800  4300
	1    0    0    -1  
$EndComp
Connection ~ 800  4300
$Comp
L E-Bike:INA290 U4
U 1 1 60CDA3F9
P 1700 5550
F 0 "U4" H 2144 5596 50  0000 L CNN
F 1 "INA290" H 2144 5505 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-353_SC-70-5" H 1900 6300 50  0001 C CNN
F 3 "https://www.ti.com/lit/ds/symlink/ina290-q1.pdf?ts=1624044330872&ref_url=https%253A%252F%252Fwww.ti.com%252Famplifier-circuit%252Fcurrent-sense%252Fanalog-output%252Fproducts.html" H 1900 6300 50  0001 C CNN
F 4 "INA290A4QDCKRQ1" H 1700 5550 50  0001 C CNN "Part Number"
	1    1700 5550
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x04 J5
U 1 1 61D1326B
P 10450 1000
F 0 "J5" H 10530 992 50  0000 L CNN
F 1 "Conn_01x04" H 10530 901 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical" H 10450 1000 50  0001 C CNN
F 3 "~" H 10450 1000 50  0001 C CNN
	1    10450 1000
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x04 J6
U 1 1 61D132ED
P 10450 1600
F 0 "J6" H 10530 1592 50  0000 L CNN
F 1 "Conn_01x04" H 10530 1501 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical" H 10450 1600 50  0001 C CNN
F 3 "~" H 10450 1600 50  0001 C CNN
	1    10450 1600
	1    0    0    -1  
$EndComp
Text Label 1400 4300 0    50   ~ 0
VBUS_R
Text Label 9900 1500 0    50   ~ 0
VBUS_R
Connection ~ 10250 1600
Wire Wire Line
	10250 1600 10250 1500
Connection ~ 10250 1700
Wire Wire Line
	10250 1700 10250 1600
Wire Wire Line
	10250 1800 10250 1700
Wire Wire Line
	9900 1500 10250 1500
Connection ~ 10250 1500
$Comp
L power:VBUS #PWR033
U 1 1 61D19CA2
P 9900 900
F 0 "#PWR033" H 9900 750 50  0001 C CNN
F 1 "VBUS" H 9915 1073 50  0000 C CNN
F 2 "" H 9900 900 50  0001 C CNN
F 3 "" H 9900 900 50  0001 C CNN
	1    9900 900 
	1    0    0    -1  
$EndComp
Wire Wire Line
	9900 900  10250 900 
Wire Wire Line
	10250 1200 10250 1100
Connection ~ 10250 900 
Connection ~ 10250 1000
Wire Wire Line
	10250 1000 10250 900 
Connection ~ 10250 1100
Wire Wire Line
	10250 1100 10250 1000
$EndSCHEMATC
