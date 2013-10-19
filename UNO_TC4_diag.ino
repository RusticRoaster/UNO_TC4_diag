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
// V0.02 Oct. 16,2013   Stan Gardner added pin toggle test
// V0.03 Oct. 16,2013   Stan Gardner added pin to pin loop test
// V0.04 Oct. 18,2013   Stan Gardner changed toggle pin text
#define BANNER_CAT "UNO_TC4_diag V0.04" // version


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
uint8_t Toggle_mode=0;
uint8_t toggle_pin=0;

//Dont mess with serial port or I2C bus
#define MAX_PIN 17
#define MIN_PIN 2

#define LOOP_BASE_PIN 2

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
  serialPrintln_P(PSTR("1 = test I2C pins, connect A2 to A4,A3 to A5"));
  serialPrintln_P(PSTR("2 = scan I2C bus, open or connect A2 to A4,A3 to A5"));
  serialPrintln_P(PSTR("3 =  I2C Loop Test, connect A2 to A4,A3 to A5"));
  serialPrintln_P(PSTR("l = arduino pin number looped to  arduino pin 2 or L enter to reset)"));
  serialPrintln_P(PSTR("T = pin number to toggle(arduino numbers 2-17 or T enter to reset)"));
  serialPrintln_P(PSTR("t = toggle pin"));
  serialPrintln_P(PSTR("v = toggle verbose debug mode"));
  serialPrintln_P(PSTR("V = show program variables"));
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
  case 'l':
      loop_test_pins();
      break;

   case 'T':
      set_toggle_pin();
    break;
  case 't':
      toggle_pins();
      break;

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
  case 'o':
    serialPrintln_P(PSTR("Not supported"));
  default:
  display_menu();
  break;
 }
  return;
}

uint8_t last_toggle = 0;
void set_toggle_pin(void){
  int temp_i = 0;
  if(strlen(command) >= 3){
    temp_i = atoi(command+2);
     if((temp_i > MAX_PIN) || (temp_i < MIN_PIN)){
          serialPrint_P(PSTR("Error, enter a number between  "));
          Serial.print(MIN_PIN);
          serialPrint_P(PSTR(" and "));
          Serial.println(MAX_PIN);          
     }
     else{       
        if(Toggle_mode && toggle_pin)
          pinMode(toggle_pin,INPUT_PULLUP);  
        toggle_pin = (uint8_t)temp_i;        
        last_toggle = 0;
        Toggle_mode=1;
        pinMode(toggle_pin,OUTPUT);
        digitalWrite(toggle_pin,0);                 
        input_accepted();
     }
  }
  else{
      last_toggle = 0;
      Toggle_mode=0;
      if(toggle_pin){
        pinMode(toggle_pin,INPUT_PULLUP);
        serialPrint_P(PSTR("Pin toggle on pin "));
        Serial.print(toggle_pin);
        serialPrintln_P(PSTR(" turned Off"));
      }
      else{
        serialPrintln_P(PSTR("Toggle still off"));
      }
      toggle_pin=0;      
  }        
  return;
}

void loop_test_pins(void){
  int test_pin = 0,rval=0,error_cnt=0;
  if(strlen(command) >= 3){
    test_pin = atoi(command+2);
     if((test_pin > MAX_PIN) || (test_pin < MIN_PIN+1)){
          serialPrint_P(PSTR("Error, enter a number between  "));
          Serial.print(MIN_PIN+1);
          serialPrint_P(PSTR(" and "));
          Serial.println(MAX_PIN);          
     }
     else{       
        pinMode(LOOP_BASE_PIN,INPUT_PULLUP);  
        pinMode(test_pin,INPUT_PULLUP);  
        rval=digitalRead(LOOP_BASE_PIN);
        if(!rval){
          serialPrintln_P(PSTR("Error, Base pin not high"));
          error_cnt++;
        }
        rval=digitalRead(test_pin);
        if(!rval){
          serialPrintln_P(PSTR("Error, test pin not high"));
          error_cnt++;
        }
        pinMode(test_pin,OUTPUT);  
        digitalWrite(test_pin,1);
        rval=digitalRead(LOOP_BASE_PIN);
        if(!rval){
          serialPrintln_P(PSTR("Error2, base pin  not high"));
          error_cnt++;
        }
        digitalWrite(test_pin,0);
        rval=digitalRead(LOOP_BASE_PIN);
        if(rval){
          serialPrintln_P(PSTR("Error, base pin not low"));
          error_cnt++;
        }

        pinMode(test_pin,INPUT_PULLUP);  
        pinMode(LOOP_BASE_PIN,OUTPUT);  
        digitalWrite(LOOP_BASE_PIN,1);
        rval=digitalRead(test_pin);
        if(!rval){
          serialPrintln_P(PSTR("Error, test input pin not high"));
          error_cnt++;
        }
        digitalWrite(LOOP_BASE_PIN,0);
        rval=digitalRead(test_pin);
        if(rval){
          serialPrintln_P(PSTR("Error, test input pin not high"));
          error_cnt++;
        }
        if(!error_cnt)
          serialPrintln_P(PSTR("Test passes"));
        pinMode(LOOP_BASE_PIN,INPUT_PULLUP);  
        pinMode(test_pin,INPUT_PULLUP);  
     }
  }
  else{
        input_error();
        serialPrintln_P(PSTR("Usage: l SingleSpace IntValue"));

  }        
  return;
}
void clear_toggle_flags(){
      last_toggle = 0;
      Toggle_mode=0;
      toggle_pin=0;
}
void toggle_pins(void){
  if(Toggle_mode){
    serialPrint_P(PSTR("Pin "));
    if(last_toggle){
      digitalWrite(toggle_pin,0);
      Serial.print(toggle_pin);
        serialPrintln_P(PSTR(" Toggled Low"));
    }
    else{
      digitalWrite(toggle_pin,1);
      Serial.print(toggle_pin);
        serialPrintln_P(PSTR(" Toggled High"));
    }
    last_toggle = !last_toggle;
  }
  else{
        serialPrintln_P(PSTR(" Toggle Pin not set"));
  }    
}

void i2c_loop_test(void){
  int tmp_cnt=0;
  if(!i2c_on){
      Wire.begin();
      i2c_on=1;
      delay(100);
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
   Serial.print(Toggle_mode);
  serialPrintln_P(PSTR(" Toggle pin mode when 1"));
   Serial.print(toggle_pin);
  serialPrintln_P(PSTR(" Pin that toggle when toggle mode = 1"));

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
    serialPrintln_P(PSTR("No I2C devices found"));

  serialPrintln_P(PSTR("Scan done"));
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

  display_menu();

  
  delay( 1800 );
  nextLoop = 2000;
  reftime = 0.001 * nextLoop; // initialize reftime to the time of first sample
  first = true;
  pinMode(A0,INPUT_PULLUP);
  pinMode(A1,INPUT_PULLUP);
  pinMode(A2,INPUT_PULLUP);
  pinMode(A3,INPUT_PULLUP);

  pinMode(2,INPUT_PULLUP);
  pinMode(3,INPUT_PULLUP);
  pinMode(4,INPUT_PULLUP);
  pinMode(5,INPUT_PULLUP);
  pinMode(6,INPUT_PULLUP);
  pinMode(7,INPUT_PULLUP);

  pinMode(8,INPUT_PULLUP);
  pinMode(9,INPUT_PULLUP);
  pinMode(10,INPUT_PULLUP);
  pinMode(11,INPUT_PULLUP);
  pinMode(12,INPUT_PULLUP);
  //led on 13  
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

