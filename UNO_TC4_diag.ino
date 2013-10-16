// UNO_TC4_diag.ino
//
// diagnostic utility program to Test the arduino base pins
// used by the TC4 shield.
//
// ------------------------------------------------------------------------------------------
// Copyright (c) 2013, Stan Gardner
// All rights reserved.
//
// Contributor:  Stan Garnder
//
// Redistribution and use in source and binary forms, with or without modification, are 
// permitted provided that the following conditions are met:
//
//   Redistributions of source code must retain the above copyright notice, this list of 
//   conditions and the following disclaimer.
//
//   Redistributions in binary form must reproduce the above copyright notice, this list 
//   of conditions and the following disclaimer in the documentation and/or other materials 
//   provided with the distribution.
//
//   Neither the name of the author nor the names of its contributors may be 
//   used to endorse or promote products derived from this software without specific prior 
//   written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS 
// OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
// MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL 
// THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS 
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// ------------------------------------------------------------------------------------------

// -----------------------------------------------------------------------------------------------
// UNO_TC4_diag Rev.s
// V0.01 Oct. 15,2013   Stan Gardner creation
#define BANNER_CAT "UNO_TC4_diag V0.01" // version


// The user.h file contains user-definable compiler options
// It must be located in the same folder as TC4_diag.ino
#include "user.h"

// this library included with the arduino distribution
#include <Wire.h>

#include <avr/pgmspace.h>
#include <alloca.h>
#include <avr/interrupt.h> 

#define MAX_COMMAND 80 // max length of a command string
#define LOOPTIME 1000 // cycle time, in ms

/*
// ------------------------ other compile directives
#define MIN_DELAY 300   // ms between ADC samples (tested OK at 270)
#define NCHAN 4  // number of TC input channels
#define DP 1  // decimal places for output on serial port
#define D_MULT 0.001 // multiplier to convert temperatures from int to float
#define MAX_COMMAND 80 // max length of a command string
*/
//----------------- user interface ----------------

uint8_t verbose_mode = 0;  //toggle to display extra debug info 
uint8_t sample_cnt = 0;
uint8_t i2c_on=0;

// used by A2, A3 pin change interrupt handler
volatile int trans_cnt=0;

//char *input_ptr;
//shadow memory of information to fill eeprom calibration


// --------------------------------------------------------------
// global variables

// used in main loop
float timestamp = 0;
boolean first;
uint32_t nextLoop;
float reftime; // reference for measuring elapsed time
char command[MAX_COMMAND+1]; // input buffer for commands from the serial port


// prototypes
void serialPrintln_P(const prog_char* s);
void serialPrint_P(const prog_char* s);
void append( char* str, char c );

void display_menu(void);
void show_variables(void);
void input_error(void);
void input_accepted(void);
void processCommand(void);  // a newline character has been received, so process the command
void checkSerial(void);  // buffer the input from the serial port
//void checkStatus( uint32_t ms ); // this is an active delay loop

#define I2C_CLK A5
#define I2C_DAT A4
#define I2C_CLK_RCV A3
#define I2C_DAT_RCV A2

// simple interrupt handler to test I2C output pins
ISR(PCINT1_vect){
  trans_cnt++;
}


void display_menu(){
  serialPrintln_P(PSTR(""));
  serialPrintln_P(PSTR("v = toggle verbose debug mode"));
  serialPrintln_P(PSTR("V = show program variables"));
  serialPrintln_P(PSTR("1 = test I2C pins, connect A2 to A4,A3 to A5"));
  serialPrintln_P(PSTR("2 = scan I2C bus, open or connect A2 to A4,A3 to A5"));
  serialPrintln_P(PSTR("3 =  I2C Loop Test, connect A2 to A4,A3 to A5"));
  serialPrintln_P(PSTR("Enter a Letter to run item"));
  return;
}
// -------------------------------------
void input_accepted(void){serialPrintln_P(PSTR("Input Accepted"));}
void input_error(void){serialPrintln_P(PSTR("Error - line too short"));}
void processCommand() {  // a newline character has been received, so process the command
//  char [MAX_COMMAND+1] cmd_buffer ="";
  double temp_f = 0.0;
  int temp_i=0;

 switch (command[0]){
  case 'V':
      show_variables();
      break;
  case 'v':
      if(verbose_mode){
        verbose_mode = 0;
        serialPrintln_P(PSTR("Verbose Mode Off"));
      }
      else{
        verbose_mode = 1;
        serialPrintln_P(PSTR("Verbose Mode On"));
      }
      break;
  case '1':
       test_pins();
        break;
  case '2':
       if(check_I2C()){
        serialPrintln_P(PSTR("I2C has trouble cant run"));
         break;
       }
      i2c_scanner();
      break;
  case '3':
       if(check_I2C()){
        serialPrintln_P(PSTR("I2C has trouble cant run"));
         break;
       }
      i2c_loop_test();
      break;

  case 'i':
  case 'l':
  case 'o':
    serialPrintln_P(PSTR("Not supported"));
  default:
  display_menu();
  break;
 }
  return;
}
void i2c_loop_test(void){
  int tmp_cnt=0;
  if(!i2c_on){
      serialPrintln_P(PSTR("I2C scan must be Ran first this test"));
      return;
    }    
      trans_cnt=0;
      PCMSK1 |= (1<<PCINT11);
      PCICR |= (1<<PCIE1);
      Wire.beginTransmission(1);
      Wire.endTransmission();
      delay(10);
      if(trans_cnt != 20){
        serialPrintln_P(PSTR("Error transistion  count not equal 20"));
      }
      debug_Print_P(PSTR("transistion count = "));
      debug_Println_int(trans_cnt);
      tmp_cnt=trans_cnt;
      PCICR &= ~(1<<PCIE1);
      PCMSK1 &= ~(1<<PCINT11);
      PCMSK1 |= (1<<PCINT10);
      PCICR |= (1<<PCIE1);
      Wire.beginTransmission(0x05);
      Wire.endTransmission();
      delay(10);
      debug_Print_P(PSTR("transistion count = "));
      debug_Println_int(trans_cnt);
      if((trans_cnt) != 27){
        serialPrintln_P(PSTR("Error transistion  count not equal 27"));
        if((tmp_cnt == 5) && (trans_cnt == 25))
          serialPrintln_P(PSTR("Loop connections may be reversed"));
      }
      else{
        serialPrintln_P(PSTR("Test pass"));
      }
      PCICR &= ~(1<<PCIE1);
      PCMSK1 &= ~(1<<PCINT10);
}
void show_variables(void){
  serialPrintln_P(PSTR("Program Information"));
   Serial.print(verbose_mode);
  serialPrintln_P(PSTR(" Verbose Debug Mode setting"));
   Serial.print(i2c_on);
  serialPrintln_P(PSTR(" I2C wire services enabled when 1"));

}
void serialPrint_P(const prog_char* s)
{
   char* p = (char*)alloca(strlen_P(s) + 1);
  strcpy_P(p, s);
  Serial.print(p);
}

void serialPrintln_P(const prog_char* s)
{
  serialPrint_P(s);
  serialPrint_P(PSTR("\n"));
}
void debug_Print_P(const prog_char* s)
{
  if(verbose_mode){
    serialPrint_P(s);
  }
}
void debug_Println_P(const prog_char* s)
{
  if(verbose_mode){
    serialPrint_P(s);
    serialPrint_P(PSTR("\n"));
  }
}
void debug_Println_int(int out)
{
  if(verbose_mode){
    Serial.println(out);
  }
}
void debug_Print_int(int out)
{
  if(verbose_mode){
    Serial.print(out);
  }
}


int check_I2C(void)
{
  int rval=0;
  if(!digitalRead(I2C_CLK)){
    serialPrintln_P(PSTR("Error I2C clock pin low"));
    rval=1;
  }    
  if(!digitalRead(I2C_DAT)){
    serialPrintln_P(PSTR("Error I2C data pin low")); 
    rval +=2;
  }
  return (rval);
}


void i2c_scanner()
{
  byte error, address;
  int nDevices;
  if(!i2c_on){
    Wire.begin();
    delay(100);
    i2c_on=1;
  }
    
  serialPrintln_P(PSTR("expecting to find nothing"));
  serialPrintln_P(PSTR("Scanning..."));

  nDevices = 0;
  for(address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      serialPrint_P(PSTR("I2C device found at address 0x"));
      if (address<16)
        serialPrint_P(PSTR("0"));
      Serial.print(address,HEX);
      serialPrintln_P(PSTR("  !"));
      nDevices++;
    }
    else if (error==4)
    {
      serialPrint_P(PSTR("Unknow error at address 0x"));
      if (address<16)
        serialPrint_P(PSTR("0"));
      Serial.println(address,HEX);
    }    
  }
  delay(5000);           // wait 5 seconds for next scan
  if (nDevices == 0)
    serialPrintln_P(PSTR("No I2C devices found\n"));
  else
    serialPrintln_P(PSTR("done\n"));
}

void test_pins(void){
  int x=0,y=0;
  if(i2c_on){
    serialPrintln_P(PSTR("Test must be run before I2C scan"));
    return;
  }    
//pins default to inputs, a0,a1 harve pulls check if no high
  x=digitalRead(I2C_CLK_RCV);
  if(!x){
    serialPrintln_P(PSTR("error CLK not High, default"));
    y++;
  }
  x=digitalRead(I2C_DAT_RCV);
  if(!x){
    serialPrintln_P(PSTR("error DAT not High, default"));
    y++;
  }
  pinMode(I2C_DAT,OUTPUT);
  pinMode(I2C_CLK,OUTPUT);

  digitalWrite(I2C_CLK,0);
  x=digitalRead(I2C_CLK_RCV);
  if(x){
    serialPrintln_P(PSTR("error CLK not Low"));
    y++;
  }

  digitalWrite(I2C_CLK,1);
  x=digitalRead(I2C_CLK_RCV);
  if(!x){
    serialPrintln_P(PSTR("error CLK not High"));
    y++;
  }

  digitalWrite(I2C_DAT,0);
  x=digitalRead(I2C_DAT_RCV);
  if(x){
    serialPrintln_P(PSTR("error DAT not Low"));
    y++;
  }

  digitalWrite(I2C_DAT,1);
  x=digitalRead(I2C_DAT_RCV);
  if(!x){
    serialPrintln_P(PSTR("error DAT not High"));
    y++;
  }
  if(!y)
    serialPrintln_P(PSTR("Test Pass"));
  pinMode(I2C_CLK,INPUT_PULLUP);
  pinMode(I2C_DAT,INPUT_PULLUP);
}

void debug_print_int(uint8_t a){
  if(verbose_mode)
    Serial.println(a,HEX);
  return;
}


// -------------------------------------
void append( char* str, char c ) { // reinventing the wheel
  int len = strlen( str );
  str[len] = c;
  str[len+1] = '\0';
}

// -------------------------------------
void checkSerial() {  // buffer the input from the serial port
  char c;
  while( Serial.available() > 0 ) {
    c = Serial.read();
    if( ( c == '\n' ) || ( strlen( command ) == MAX_COMMAND ) ) { // check for newline, or buffer overflow
      processCommand();
      strcpy( command, "" ); // empty the buffer
    } // end if
    else if( c != '\r' ) { // ignore CR for compatibility with CR-LF pairs
//      append( command, toupper(c) );
      append( command, c );
    } // end else
  } // end while
}

// ----------------------------------
void checkStatus( uint32_t ms ) { // this is an active delay loop
  uint32_t tod = millis();
  while( millis() < tod + ms ) {
  }
}

  
// ------------------------------------------------------------------------
// MAIN
//
void setup()
{
  delay(500);
  
  Serial.begin(BAUD);
  delay(500);
  serialPrintln_P(PSTR(BANNER_CAT));
//  Wire.begin(); 

  display_menu();

  
  delay( 1800 );
  nextLoop = 2000;
  reftime = 0.001 * nextLoop; // initialize reftime to the time of first sample
  first = true;
  pinMode(A2,INPUT_PULLUP);
  pinMode(A3,INPUT_PULLUP);
  
//MCUCR = (1<<ISC01) | (1<<ISC00);
}

// -----------------------------------------------------------------
void loop() {
  float idletime;
  uint32_t thisLoop;

  // delay loop to force update on even LOOPTIME boundaries
  while ( millis() < nextLoop ) { // delay until time for next loop
      checkSerial(); // Has a command been received?
   }
  while(sample_cnt){
    sample_cnt--;
    thisLoop = millis(); // actual time marker for this loop
    timestamp = 0.001 * float( thisLoop ) - reftime; // system time, seconds, for this set of samples

    idletime = LOOPTIME - ( millis() - thisLoop );
    // arbitrary: complain if we don't have at least 50mS left
    if (idletime < 50 ) {
      serialPrint_P(PSTR("# idle: "));
      Serial.println(idletime);
    }
  }
  nextLoop += LOOPTIME; // time mark for start of next update 
}

