#include <SPI.h>

/* ADS1248 Register (see p42 for Register Map) */
#define   MUX0      0x00  //Multiplexer Control Register 0
#define   VBIAS     0X01  //Bias Voltage Register
#define   MUX1      0x02  //Multiplexer Control REgister 1
#define   SYS0      0x03  //System Control Register 0
#define   OFC0      0X04  //Offset Calibration Coefficient Register 0
#define   OFC1      0x05  //Offset Calibration Coefficient Register 1
#define   OFC2      0x06  //Offset Calibration Coefficient Register 2
#define   FSC0      0x07  //Full scale Callibration Coefficient Register 0
#define   FSC1      0x08  //Full scale Callibration Coefficient Register 1
#define   FSC2      0x09  //Full scale Callibration Coefficient REgister 2
#define   IDAC0     0x0A  //IDAC Control Register 0
#define   IDAC1     0x0B  //IDAC Control Register 1
#define   GPIOCFG   0x0C  //GPIO Configuration Register
#define   GPIODIR   0x0D  //GPIODirection REgister
#define   GPIODAT   0x0E  //GPIO Data Register

/* MUX0 - Multiplexer Control Register 0 (see p43 - bring together with bitwise OR | */
/* BIT7 - BIT6 -  BIT5   -  BIT4   -  BIT3   -  BIT2   -  BIT1   -  BIT0 */
/* BCS1 - BCS0 - MUX_SP2 - MUX_SP1 - MUX_SP0 - MUX_SN2 - MUX_SN1 - MUXSN0 */
#define   MUX0_RESET  0x01      // Reset MUX0 Register 
/* BCS1:0 These bits select the magnitude of the sensor detect current source */
#define   BCS1_1      B00000000    // 00 Burnout current source off (default)
#define   BCS1_2      B01000000    // 01 Burnout current source on 0.5 �A
#define   BCS1_3      B10000000    // 10 Burnout current source on 2 �A
#define   BCS1_4      B11000000    // 11 Burnout current source on 10 �A
/* MUX_SP2:0 Positive input channel selection bits */
#define   MUX_SP2_AIN0    B00000000    // 000 AIN0 (default)
#define   MUX_SP2_AIN1    B00001000    // 001 AIN1
#define   MUX_SP2_AIN2    B00010000    // 010 AIN2
#define   MUX_SP2_AIN3    B00011000    // 011 AIN3
#define   MUX_SP2_AIN4    B00100000    // 100 AIN4
#define   MUX_SP2_AIN5    B00101000    // 101 AIN5
#define   MUX_SP2_AIN6    B00110000    // 110 AIN6
#define   MUX_SP2_AIN7    B00111000    // 111 AIN7
/* MUX_SN2:0 Negative input channel selection bits */
#define   MUX_SN2_AIN0    B00000000    // 000 AIN0
#define   MUX_SN2_AIN1    B00000001    // 001 AIN1 (default)
#define   MUX_SN2_AIN2    B00000010    // 010 AIN2
#define   MUX_SN2_AIN3    B00000011    // 011 AIN3
#define   MUX_SN2_AIN4    B00000100    // 100 AIN4
#define   MUX_SN2_AIN5    B00000101    // 101 AIN5
#define   MUX_SN2_AIN6    B00000110    // 110 AIN6
#define   MUX_SN2_AIN7    B00000111    // 111 AIN7

/* VBIAS - Bias Voltage Register (see p43 - bring together with bitwise OR | */
/*  BIT7  -  BIT6  -  BIT5  -  BIT4  -  BIT3  -  BIT2  -  BIT1  -  BIT0  */
/* VBIAS7 - VBIAS6 - VBIAS5 - VBIAS4 - VBIAS3 - VBIAS2 - VBIAS1 - VBIAS0 */

/* VBIAS These bits apply a bias voltage of midsupply (AVDD + AVSS)/2 to the selected analog input */
#define   VBIAS_RESET 0x00     // Reset VBIAS Register 
#define   VBIAS_7   B10000000    // AIN7
#define   VBIAS_6   B01000000    // AIN6
#define   VBIAS_5   B00100000    // AIN5
#define   VBIAS_4   B00010000    // AIN4
#define   VBIAS_3   B00001000    // AIN3
#define   VBIAS_2   B00000100    // AIN2
#define   VBIAS_1   B00000010    // AIN1
#define   VBIAS_0   B00000001    // AIN0

/* MUX1 - Multiplexer Control Register 1 (see p44 - bring together with bitwise OR | */
/*  BIT7   -   BIT6   -   BIT5   -   BIT4   -   BIT3   -  BIT2   -  BIT1   -  BIT0 */
/* CLKSTAT - VREFCON1 - VREFCON0 - REFSELT1 - REFSELT0 - MUXCAL2 - MUXCAL1 - MUXCAL0 */
#define      MUX1_RESET    0x00      // Reset MUX1 Register 
/* CLKSTAT This bit is read-only and indicates whether the internal or external oscillator is being used
0 = internal, 1 = external */
/* VREFCON1 These bits control the internal voltage reference. These bits allow the reference to be turned on or
off completely, or allow the reference state to follow the state of the device. Note that the internal
reference is required for operation the IDAC functions.*/
#define   VREFCON1_OFF    B00000000    // Internal reference is always off (default)
#define   VREFCON1_ON     B00100000    // Internal reference is always on
#define   VREFCON1_PS     B01100000    // Internal reference is on when a conversion is in progress
/* REFSELT1:0 These bits select the reference input for the ADC  */
#define   REFSELT1_REF0   B00000000    // REF0 input pair selected (default)
#define   REFSELT1_REF1   B00001000    // REF1 input pair selected
#define   REFSELT1_ON     B00010000    // Onboard reference selected
#define   REFSELT1_ON_REF0  B00011000  // Onboard reference selected and internally connected to REF0 input pair
/* MUXCAL2:0 These bits are used to select a system monitor. The MUXCAL selection supercedes selections from registers MUX0 and MUX1  */
#define   MUXCAL2_NORMAL  B00000000    // Normal operation (default)
#define   MUXCAL2_OFFSET  B00000001    // Offset measurement
#define   MUXCAL2_GAIN    B00000010    // Gain measurement
#define   MUXCAL2_TEMP    B00000011    // Temperature diode
#define   MUXCAL2_REF1    B00000100    // External REF1 measurement
#define   MUXCAL2_REF0    B00000101    // External REF0 measurement
#define   MUXCAL2_AVDD    B00000110    // AVDD measurement
#define   MUXCAL2_DVDD    B00000111    // DVDD measurement

/* SYS0 - System Control Register 0 (see p45 - bring together with bitwise OR | */
/* BIT7 - BIT6 - BIT5 - BIT4 - BIT3 - BIT2 - BIT1 - BIT0 */
/*  0   - PGA2 - PGA1 - PGA0 - DOR3 - DOR2 - DOR1 - DOR0 */
/* PGA2:0 These bits determine the gain of the PGA  */
#define   PGA2_0      B00000000    // 1 (default)
#define   PGA2_2      B00010000    // 2
#define   PGA2_4      B00100000    // 4
#define   PGA2_8      B00110000    // 8
#define   PGA2_16     B01000000    // 16
#define   PGA2_32     B01010000    // 32
#define   PGA2_64     B01100000    // 64
#define   PGA2_128    B01110000    // 128
/* DOR3:0 These bits select the output data rate of the ADC  */
#define   DOR3_5      B00000000    // 5SPS (default)
#define   DOR3_10     B00000001    // 10SPS 
#define   DOR3_20     B00000010    // 20SPS 
#define   DOR3_40     B00000011    // 40SPS 
#define   DOR3_80     B00000100    // 80SPS 
#define   DOR3_160    B00000101    // 160SPS 
#define   DOR3_320    B00000110    // 320SPS 
#define   DOR3_640    B00000111    // 640SPS 
#define   DOR3_1000   B00001000    // 1000SPS 
#define   DOR3_2000   B00001001    // 2000SPS 

/* IDAC0- IDAC Control Register 0 (see p46 - bring together with bitwise OR | */
/* BIT7 - BIT6 - BIT5 - BIT4 -   BIT3    - BIT2  - BIT1  - BIT0 */
/* ID3  - ID2  - ID1  - ID0  - DRDY MODE - IMAG2 - IMAG1 - IMAG0 */
#define IDAC0_ID  144 
/* DRDY MODE This bits sets the DOUT/DRDY pin functionality  */
#define   DRDY_ONLY     B00000000    // Data out only (default)
#define   DRDY_BOTH     B00001000    // Data out and Data ready functionality
/* IMAG2:0 The ADS1248 have two programmable current source DACs that can be used for sensor excitation.
The IMAG bits control the magnitude of the excitation current. The IDACs require the internal reference to be on.  */
#define   IMAG2_OFF     B00000000    // off (default)
#define   IMAG2_50      B00000001    // 50uA
#define   IMAG2_100     B00000010    // 100uA
#define   IMAG2_250     B00000011    // 250uA
#define   IMAG2_500     B00000100    // 500uA
#define   IMAG2_750     B00000101    // 750uA
#define   IMAG2_1000    B00000110    // 1000uA
#define   IMAG2_1500    B00000111    // 1500uA

/* IDAC1- IDAC Control Register 1 (see p47 - bring together with bitwise OR | */
/*  BIT7  -  BIT6  -  BIT5  -  BIT4  - BIT3   -  BIT2  -  BIT1  -  BIT0 */
/* I1DIR3 - I1DIR2 - I1DIR2 - I1DIR0 - I2DIR3 - I2DIR2 - I2DIR1 - I2DIR0 */
/* I1DIR3:0 These bits select the output pin for the first current source DAC  */
#define   I1DIR_AIN0     B00000000    // AIN0
#define   I1DIR_AIN1     B00010000    // AIN1
#define   I1DIR_AIN2     B00100000    // AIN2
#define   I1DIR_AIN3     B00110000    // AIN3
#define   I1DIR_AIN4     B01000000    // AIN4
#define   I1DIR_AIN5     B01010000    // AIN5
#define   I1DIR_AIN6     B01100000    // AIN6
#define   I1DIR_AIN7     B01110000    // AIN7 
#define   I1DIR_IEXT1    B10000000    // IEXT1  
#define   I1DIR_IEXT2    B10010000    // IEXT2
#define   I1DIR_OFF      B11000000    // Disconnected (default)

/* I2DIR3:0 These bits select the output pin for the second current source DAC  */
#define   I2DIR_AIN0     B00000000    // AIN0
#define   I2DIR_AIN1     B00000001    // AIN1
#define   I2DIR_AIN2     B00000010    // AIN2
#define   I2DIR_AIN3     B00000011    // AIN3
#define   I2DIR_AIN4     B00000100    // AIN4
#define   I2DIR_AIN5     B00000101    // AIN5
#define   I2DIR_AIN6     B00000110    // AIN6
#define   I2DIR_AIN7     B00000111    // AIN7 
#define   I2DIR_IEXT1    B00001000    // IEXT1  
#define   I2DIR_IEXT2    B00001001    // IEXT2
#define   I2DIR_OFF      B00001100    // Disconnected (default)

/* SPI COMMAND DEFINITIONS (p49) */
/*SYSTEM CONTROL */
#define   WAKEUP    0x00  //Exit Sleep Mode
#define   SLEEP     0x01  //Enter Sleep Mode
#define   SYNC      0x04  //Synchornize the A/D Conversion
#define   RESET     0x06  //Reset To Power UP values
#define   NOP       0xFF  //NO Operation
/*DATA READ*/
#define   RDATA     0x12  //Read data once
#define   RDATAC    0x14  //Read data continously
#define   SDATAC    0x16  //Stop reading data continously
/*READ REGISTER */
#define   RREG      0x20  //Read From Register
#define   WREG      0x40  //Write To Register
/*Calibration */
#define   SYSOCAL   0x60  //System Offset Calibration
#define   SYSGCAL   0x61  //System Gain Calibration
#define   SELFOCAL  0x62  //Self Offset Calibration

#define SPI_SPEED 4000000

// ----------------------------------------------------

// set I/O pins used in addition to clock, data in, data out
const byte CS = 77;             // digital pin 10 for /CS
const byte START = 34;          // PC2
const byte DRDY = 35;           // PC3

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  delay(1000);

  pinMode(CS, OUTPUT);              // Set the slaveSelectPin as an output  
  pinMode(START, OUTPUT);
  pinMode(DRDY, INPUT);             
  
  digitalWrite(CS, HIGH);           // CS HIGH = nothing selected  
  delay(100);
  digitalWrite(START, HIGH);
  delay(100);  

  SPI.begin();
  
  ADS1248_Reset();  

  delay(100);
}

void loop() {
  // put your main code here, to run repeatedly:
  
  // Check_Supplied_Voltage();
  // Check_Internal_Temp();
  // Check_Rre0_Voltage();
  
  Read_ADS1248(1);   
  Read_ADS1248(2);   
  Read_ADS1248(3); 
 
  Serial.print("\n");
  delay(100);
}

void Read_ADS1248(uint8_t Ch_Num) {  
  
  // register int i = Ch_Num;
  unsigned long Volt_Val = 0;
  double Voltage;

  // AIN0을 기준전압으로 설정
  ADS1248_SetRegisterValue(VBIAS, VBIAS_0);
  
  switch (Ch_Num) {
    case 0:
        Serial.println("AIN0 is Bais Voltage");
        break;
    case 1:
        // AIN1을 선택
        ADS1248_SetRegisterValue(MUX0, BCS1_1|MUX_SP2_AIN1);
        Volt_Val = 0;
        Volt_Val = ADS1248_GetConversion();
        Voltage = ((2.5 / 16777216) * Volt_Val);
        Voltage *= 4 * 2.5;
        
        Serial.print("AIN1 Voltage : ");
        Serial.print(Voltage);
        Serial.println("V");
        break;
    case 2:
        // AIN2을 선택
        ADS1248_SetRegisterValue(MUX0, BCS1_1|MUX_SP2_AIN2);
        Volt_Val = 0;
        Volt_Val = ADS1248_GetConversion();
        Voltage = ((2.5 / 16777216) * Volt_Val);
        Voltage *= 4 * 2.5;
        
        Serial.print("AIN2 Voltage : ");
        Serial.print(Voltage);
        Serial.println("V");
        break;
    case 3:
        // AIN1을 선택
        ADS1248_SetRegisterValue(MUX0, BCS1_1|MUX_SP2_AIN3);
        Volt_Val = 0;
        Volt_Val = ADS1248_GetConversion();
        Voltage = ((2.5 / 16777216) * Volt_Val);
        Voltage *= 4 * 2.5;
        
        Serial.print("AIN3 Voltage : ");
        Serial.print(Voltage);
        Serial.println("V");
        break;
    case 4:
        ADS1248_SetRegisterValue(MUX0, BCS1_1|MUX_SP2_AIN4);
        Volt_Val = 0;
        Volt_Val = ADS1248_GetConversion();
        Voltage = ((2.5 / 16777216) * Volt_Val);
        Voltage *= 4 * 2.5;
        
        Serial.print("AIN4 Voltage : ");
        Serial.print(Voltage);
        Serial.println("V");
        break;
    case 5:
        ADS1248_SetRegisterValue(MUX0, BCS1_1|MUX_SP2_AIN5);
        Volt_Val = 0;
        Volt_Val = ADS1248_GetConversion();
        Voltage = ((2.5 / 16777216) * Volt_Val);
        Voltage *= 4 * 2.5;
        
        Serial.print("AIN5 Voltage : ");
        Serial.print(Voltage);
        Serial.println("V");
        break;
    case 6:
        ADS1248_SetRegisterValue(MUX0, BCS1_1|MUX_SP2_AIN6);
        Volt_Val = 0;
        Volt_Val = ADS1248_GetConversion();
        Voltage = ((2.5 / 16777216) * Volt_Val);
        Voltage *= 4 * 2.5;
        
        Serial.print("AIN6 Voltage : ");
        Serial.print(Voltage);
        Serial.println("V");
        break;
    case 7:
        ADS1248_SetRegisterValue(MUX0, BCS1_1|MUX_SP2_AIN7);
        Volt_Val = 0;
        Volt_Val = ADS1248_GetConversion();
        Voltage = ((2.5 / 16777216) * Volt_Val);
        Voltage *= 4 * 2.5;
        
        Serial.print("AIN7 Voltage : ");
        Serial.print(Voltage);
        Serial.println("V");
        break;      
    default:
        break;
    
  }
  
  /* 
  // AIN0을 기준전압으로 설정
  ADS1248_SetRegisterValue(MUX0, BCS1_1|MUX_SP2_AIN1);  
  ADS1248_SetRegisterValue(VBIAS, VBIAS_0);  
  delay(500);
  
  Volt_Val = 0;
  Volt_Val = ADS1248_GetConversion();
  // Serial.print  ("AIN1 : ");
  // Serial.print(Volt_Val, HEX);
  // Serial.print  ("  ");
  // Serial.print(Volt_Val, DEC);
  // Serial.println("  ");
  // double voltage = volt_val / 16777216;  
  Voltage = ((2.5 / 16777216) * Volt_Val);
  Voltage *= 4 * 2.5;
  
  Serial.print("AIN1 Voltage : ");
  Serial.print(Voltage);
  Serial.println("V");  

  ADS1248_SetRegisterValue(MUX0, BCS1_1|MUX_SP2_AIN2);    
  delay(500);

  Volt_Val = 0;
  Volt_Val = ADS1248_GetConversion();
  // Serial.print  ("AIN2 : ");
  // Serial.print(Volt_Val, HEX);
  // Serial.print  ("  ");
  // Serial.print(Volt_Val, DEC);
  // Serial.println("  ");
  Voltage = ((2.5 / 16777216) * Volt_Val);
  Voltage *= 4 * 2.5;

  Serial.print("AIN2 Voltage (Bias) : ");
  Serial.print(Voltage);
  Serial.println("V");

  ADS1248_SetRegisterValue(MUX0, BCS1_1|MUX_SP2_AIN3);    
  delay(500);

  Volt_Val = 0;
  Volt_Val = ADS1248_GetConversion();
  // Serial.print  ("AIN3 : ");
  // Serial.print(Volt_Val, HEX);
  // Serial.print  ("  ");
  // Serial.print(Volt_Val, DEC);
  // Serial.println("\n");
  Voltage = ((2.5 / 16777216) * Volt_Val);
  Voltage *= 4 * 2.5;

  Serial.print("AIN3 Voltage : ");
  Serial.print(Voltage);
  Serial.println("V\n");
*/
}

// function to get a 3byte conversion result from the adc
long ADS1248_GetConversion() {
  
  int32_t regData;;

  digitalWrite(CS, LOW);      // Pull SS Low to Enable Communications with ADS1247
  delay(200);                  // RD: Wait 25ns for ADC12xx to get ready  
    
  /* if(digitalRead(DRDY) == 0){  // Wait until DRDY is LOW
    Serial.println("Wait for DRDY");
    return 0;
  };*/
  while (digitalRead(DRDY) == HIGH) Serial.println("Wait Ready PIN");
  
  SPI.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE1));
  // digitalWrite(CS, LOW);      // Pull SS Low to Enable Communications with ADS1247
  // delayMicroseconds(10);      // RD: Wait 25ns for ADC12xx to get ready  
  
  SPI.transfer(RDATA);        // Issue RDATA
  delayMicroseconds(10);
  
  regData |= SPI.transfer(NOP);
  //delayMicroseconds(10);
  regData <<= 8;
  regData |= SPI.transfer(NOP);
  //delayMicroseconds(10);
  regData <<= 8;
  regData |= SPI.transfer(NOP);
  delay(10);
  SPI.endTransaction();  
  
  digitalWrite(CS, HIGH);
  delay(10);

  return regData;
}

void ADS1248_Reset() {
  SPI.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE1)); // initialize SPI with  clock, MSB first, SPI Mode1
  digitalWrite(CS, LOW);
  delay(10);
  SPI.transfer(RESET);      // Reset
  delay(10);                 // Minimum 0.6ms required for Reset to finish.
  SPI.transfer(SDATAC);     // Issue SDATAC
  delay(10);
  digitalWrite(CS, HIGH);
  delay(10);
  SPI.endTransaction();
}

void ADS1248_SetRegisterValue(uint8_t regAddress, uint8_t regValue) {

  // Serial.println("SetRegisterValue");
  
  if (regAddress == IDAC0) {
    regValue = regValue | IDAC0_ID;   // add non 0 non-write register value IDAC0_ID
  }
  
  uint8_t regValuePre = ADS1248_GetRegisterValue(regAddress);

  if (regValue != regValuePre) {
    //digitalWrite(_START, HIGH);
    delayMicroseconds(10);
    //waitforDRDY();
    SPI.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE1)); // initialize SPI with SPI_SPEED, MSB first, SPI Mode1
    digitalWrite(CS, LOW);
    delayMicroseconds(10);
    

    SPI.transfer(SDATAC);           // Issue SDATAC
    delayMicroseconds(10);

    SPI.transfer(WREG | regAddress); // send 1st command byte, address of the register
    SPI.transfer(0x00);             // send 2nd command byte, write only one register
    SPI.transfer(regValue);         // write data (1 Byte) for the register
    delayMicroseconds(10);

    digitalWrite(CS, HIGH);
    //digitalWrite(_START, LOW);
    delayMicroseconds(10);


    if (regValue != ADS1248_GetRegisterValue(regAddress)) {   // Check if write was succesfull
      Serial.print("Write to Register 0x");
      Serial.print(regAddress, HEX);
      Serial.println(" failed!");
    }
    SPI.endTransaction();    
  }
  
  // Serial.print("Check regAddress : ");
  // Serial.print(regAddress,HEX);
  // Serial.print(" / regValue : ");
  // Serial.println(ADS1248_GetRegisterValue(regAddress), HEX);
}

unsigned long ADS1248_GetRegisterValue(uint8_t regAddress) {
  uint8_t bufr;
  //digitalWrite(_START, HIGH);
  //waitforDRDY();
  SPI.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE1)); // initialize SPI with 4Mhz clock, MSB first, SPI Mode0
  digitalWrite(CS, LOW);
  delayMicroseconds(10);
  SPI.transfer(RREG | regAddress); // send 1st command byte, address of the register
  SPI.transfer(0x00);     // send 2nd command byte, read only one register
  delayMicroseconds(10);
  bufr = SPI.transfer(NOP); // read data of the register
  delayMicroseconds(10);
  digitalWrite(CS, HIGH);
  //digitalWrite(_START, LOW);
  SPI.endTransaction();
  return bufr;
}

// This function measures the supplied voltages
void Check_Supplied_Voltage(void) {
  ADS1248_SetRegisterValue(MUX1, MUXCAL2_DVDD | VREFCON1_ON | REFSELT1_ON);
  // Serial.println("Wait for Measurement:");
  delay(500);
  unsigned long volt_val = ADS1248_GetConversion();
  // Serial.println(volt_val, DEC);
  double voltage = (2.048 / 16777216)*volt_val;
  voltage *= 4 * 2.048;
  Serial.print("DVDD Voltage: ");
  Serial.print(voltage);
  Serial.print("V / ");

  ADS1248_SetRegisterValue(MUX1, MUXCAL2_AVDD | VREFCON1_ON | REFSELT1_ON);
  // Serial.println("Wait for Measurement:");
  delay(500);
  
  unsigned long volt1_val = ADS1248_GetConversion();
  // Serial.println(volt1_val, DEC);
  double voltage1 = (2.048 / 16777216)*volt1_val;
  voltage1 *= 4 * 2.048;
  Serial.print("AVDD Voltage: ");
  Serial.print(voltage1);
  Serial.println("V");
}

void Check_Internal_Temp(void) {
  ADS1248_SetRegisterValue(MUX1, MUXCAL2_TEMP | VREFCON1_ON | REFSELT1_ON);
  // Serial.println("Wait for Measurement:");
  delay(500);
  
  unsigned long temp_val = ADS1248_GetConversion();
  // Serial.println(temp_val, DEC);
  double temp_voltage = (2.048 / 8388608)*temp_val; //only 23bit data here.
  // Serial.println(temp_voltage, DEC);
  double temp = ((temp_voltage - 0.118) / (405 * 0.000001)) + 25; //see Datasheet S.30
  Serial.print("Internal Temperature is ");
  Serial.print(temp);
  Serial.println(" °C");
}

void Check_Rre0_Voltage(void) {
  ADS1248_SetRegisterValue(MUX1, REFSELT1_ON | VREFCON1_ON | MUXCAL2_REF0);   // ADS Reference on Intern, Internal Reference on, System Montitor on REF0
  ADS1248_SetRegisterValue(IDAC0, IMAG2_1500);                                // IDAC at 1,5mA current
  ADS1248_SetRegisterValue(IDAC1, I1DIR_AIN0 | I2DIR_OFF);                    // IDAC1 Currentoutput on AIN0, IDAC2 off
  ADS1248_SetRegisterValue(SYS0, DOR3_5);
  delay(500);
  
  unsigned long volt_val = ADS1248_GetConversion();
  //Serial.println(volt_val, DEC);
  double voltage = (2.048 / (16777216))*volt_val;
  voltage *= 4 * 2.048;
  Serial.print("External V_Ref0: ");
  Serial.print(voltage, DEC);
  Serial.println("V\n");

}
