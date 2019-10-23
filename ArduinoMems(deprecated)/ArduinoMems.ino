//#define REPLY
//#define PANDA // Uncomment if using the Lattepanda arduino (rather than a genuine Leonardo board)
#include <SPI.h>
#include "testclass.h"

/* Lattepanda arduino script for controlling Quarc experiment
 * MEMS: Send serial command:
 *          "M", <mems x>, <mems y>
 *          and the mems will move to that position.
 *          <mems x> and <mems y> are 16 bit unsigned ints representing a the voltage on the +
 *          electrode (the - electrode is the mirror of this about the origin). 32768 is the origin.          
 */

// Various stuff for reading from serial.
const byte num_Chars = 7;
uint8_t received_Chars[num_Chars];
uint8_t echo_Chars[num_Chars];
byte recvd_Count;
byte recvd_Byte;
byte i;

// Data from serial, arg0 is command, arg1,2,3 is data.
char arg0;
uint16_t arg1, arg2, arg3;

// Stuff related to MEMS
unsigned short f_Clk    = 140;  // Cutoff frequency for LPF on MEMS driver (read from datasheet)
byte v_Difference_Max   = 155;  // From datasheet.
uint16_t v_Diff_Max_ADC;        // How this translates into DAC values.
uint16_t v_ADC_Margin;          // Closest the voltage can be to either extreme of its range.
byte v_Bias_Volts       = 80;   // The zero position bias voltage for MEMS
uint16_t v_Bias_DAC;            // How this translates into DAC values.
byte spi_cmd_addr       = 0;    // First byte of SPI data is a command, init to 0 to put it into a known state.
uint32_t spi_Buffer     = 0;    // 24 bits of this form commands to ADC chip

#ifdef PANDA
// If this is being flashed onto lattepanda microcontroller

byte mems_En_Pin        = 22;    // MEMS drivers only active if this pin is HIGH
byte f_Clk_Pin          = 21;   // Arduino GPIO pin for f_Clk frequency.
byte spi_Sync_Pin       = 12;   // See AD5664 datasheet for Sync pin function.
#else
// If this is being flashed onto an Arduino Leonardo.
byte prog_Pin           = 7;
byte mems_En_Pin        = 3;
byte f_Clk_Pin          = 5;
byte spi_Sync_Pin       = 4;
#endif //ifdef PANDA

// AD5664 commands.
byte WRITE_TO_DAC             = 0b000;
byte UPDATE_DAC               = 0b001;
byte WRITE_TO_DAC_UPDATE_ALL  = 0b010;
byte WRITE_UPDATE_DAC         = 0b011;
byte DAC_POWER_DOWN           = 0b100;
byte DAC_RESET                = 0b110;
byte LDAC_SETUP               = 0b110;
byte INT_REF_SETUP            = 0b111;
// AD5664 addresses.
byte DAC_A    = 0b000;
byte DAC_B    = 0b001;
byte DAC_C    = 0b010;
byte DAC_D    = 0b011;
byte DAC_ALL  = 0b111;

void setup() {
  // Incoming data from serial from PC
  Serial.begin(115200, SERIAL_8N1); // Try 57600 if 115200 causes errors?
  Serial.setTimeout(20); // Don't timeout too soon!

  // MEMS Stuff.
  v_Bias_DAC = ((float)32768 / (float)80) * v_Bias_Volts;
  v_Diff_Max_ADC = ((float)32768 / (float)80) * v_Difference_Max;
  v_ADC_Margin = (65535 - v_Diff_Max_ADC) / 2; // How far values can either be from 0 or 65535. 
  v_ADC_Margin *= 2; //Double it for safety.
  // Bring up SPI link to Picoamp here
  pinMode(mems_En_Pin, OUTPUT);
  digitalWrite(mems_En_Pin, LOW); // High voltage is definitely off now.
  pinMode(spi_Sync_Pin, OUTPUT);
  digitalWrite(spi_Sync_Pin, HIGH); // Rising edge writes shift registers to DACs?
  pinMode(prog_Pin, OUTPUT);
  digitalWrite(prog_Pin, HIGH);
  
  SPI.begin();
  // SPI_MODE0 CPOL=0, CPHA=0, OE=falling, DC=rising
  // SPI_MODE1 CPOL=0, CPHA=1, OE=rising, DC=falling
  // SPI_MODE2 CPOL=1, CPHA=0, OE=rising, DC=falling
  // SPI_MODE3 CPOL=1, CHPA=1, )E=falling, DC=rising
  SPI.beginTransaction(SPISettings(24000000, MSBFIRST, SPI_MODE0));
  
  // Cutoff frequency for MEMS mirror
  tone(f_Clk_Pin, f_Clk * 60);

  //InitMems();
}

void loop() {
  // put your main code here, to run repeatedly:
  
//  if (Serial.available() > 0){
//    //recvd_Count = Serial.readBytes(received_Chars, num_Chars);
//    recvd_Count = 0;
//    while (recvd_Count < num_Chars){
//      recvd_Byte = Serial.read();
//      received_Chars[recvd_Count] = recvd_Byte;
//      recvd_Count++;
//    }


  if (Serial.available() > 0){
      //Packet is valid.
      if(Serial.readBytes(received_Chars, num_Chars) == num_Chars){
        arg0 = received_Chars[0];
        arg1 = ((uint16_t)received_Chars[1] << 8) + ((uint16_t)received_Chars[2]&0xFF);
        arg2 = ((uint16_t)received_Chars[3] << 8) + ((uint16_t)received_Chars[4]&0xFF);
        arg3 = ((uint16_t)received_Chars[5] << 8) + ((uint16_t)received_Chars[6]&0xFF);
  
        #ifdef REPLY
          echo_Chars[0] = arg0;
          echo_Chars[1] = arg1 >> 8;
          echo_Chars[2] = arg1 &0xFF;
          echo_Chars[3] = arg1 >> 8;
          echo_Chars[4] = arg1 &0xFF;
          echo_Chars[5] = arg1 >> 8;
          echo_Chars[6] = arg1 &0xFF;
          Serial.write(echo_Chars, num_Chars);
        #endif
        Serial.flush();
        
        if (arg0 == 'm'){
          Move_MEMS(arg1, arg2);
        }    
    
        else if (arg0 == 'I'){
        // Bring MEMS mirror up (done in Setup but might be off manually (see if arg0 == 'X')
          InitMems();
        }
        else if (arg0 == 'X'){
        // Set MEMS driver such that mirror can be removed. (back to origin, drivers off)
          RemoveMems();
        }
    }
  }
  // Zero out buffer
  for(int i = 0; i < 7; i++){
    received_Chars[i] = 0;
  }
}

void InitMems(){
  digitalWrite(spi_Sync_Pin, HIGH); // Make sure sync pin is high before first transfer.
  delay(0.1);
  // Setup commands, (from Picoamp datasheet)
  SendSPIString(0b101, 0b000, 0b1); // FULL RESET (0x280001)
  SendSPIString(0b111, 0b000, 0b1); // ENABLE INTERNAL REFERENCE (0x380001);
  SendSPIString(0b100, 0b000, 0b1111); // ENABLE ALL DAC CHANNELS (0x20000F)
  SendSPIString(0b110, 0b000, 0b0000); // SET LDAC MODE (0x300000)

  // Set MEMS to origin here.
  digitalWrite(mems_En_Pin, HIGH); // Turn on the HV drivers.
  SendSPIString(WRITE_TO_DAC_UPDATE_ALL, DAC_ALL, v_Bias_DAC);
}

void RemoveMems(){
  // Return MEMS to origin.
  SendSPIString(WRITE_TO_DAC_UPDATE_ALL, DAC_ALL, v_Bias_DAC);
  // Give it a chance to get there
  delay(500);
  // Turn off HV drivers.
  digitalWrite(mems_En_Pin, LOW);
}

int SendSPIString(byte cmd, byte addr, uint16_t data){
  // is this the right way round? See pg 21 of the AD5664 data sheet.
  spi_cmd_addr = ((cmd & 0x7) << 3) + (addr & 0x7);
  digitalWrite(spi_Sync_Pin, LOW);
  delay(0.1);
  SPI.transfer(spi_cmd_addr);
  SPI.transfer16(data);
  delay(0.1);
  digitalWrite(spi_Sync_Pin, HIGH);
//  Serial.print(((uint32_t)spi_cmd_addr << 16) + data);
  //Serial.println(data);
  return 0;
}

int Move_MEMS(uint16_t x_Plus_Value, uint16_t y_Plus_Value){
  if (x_Plus_Value > (65535 - v_ADC_Margin)){
    x_Plus_Value = 65535 - v_ADC_Margin;
  }
  else if (x_Plus_Value < v_ADC_Margin){
    x_Plus_Value = v_ADC_Margin;
  }
  if (y_Plus_Value > (65535 - v_ADC_Margin)){
    y_Plus_Value = 65535 - v_ADC_Margin;
  }
  else if (y_Plus_Value < v_ADC_Margin){
    y_Plus_Value = v_ADC_Margin;
  }

  // I guess which channel is + and - for each channel is kind of arbitary?
  SendSPIString(WRITE_TO_DAC, DAC_A, x_Plus_Value); // X+
  SendSPIString(WRITE_TO_DAC, DAC_B, 65535 - x_Plus_Value); // X-
  
  SendSPIString(WRITE_TO_DAC, DAC_C, 65535 - y_Plus_Value); // Y-
  SendSPIString(WRITE_TO_DAC, DAC_D, y_Plus_Value); //Y+
  
  SendSPIString(UPDATE_DAC, DAC_ALL, 0);
  return 0;
}
