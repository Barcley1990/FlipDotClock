/* FLIP DOT DISPLAY
 * uC: AtMega 32u4
 * 
 * Tobias Nuss
 * 11.03.2017
 */

/****************************
 * Following Commands are available:
 * 'fill'   - Fill whole Display with Dots 
 * 'clear'  - Clear whole Display
 * 'SET'    - Set new time
 * 'READ'   - Read the current stored time
 ****************************/


#define F_CPU 16000000UL
#include <avr/wdt.h>
#include <util/delay.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <DSRTCLib.h>
#include "font.h"
#include "font2.h"

#define SWITCH_TIME 150
// FP2800 Column
#define SET_HIGH  1 
#define SET_LOW   0
#define DDR_COL   DDRF
#define PORT_COL  PORTF
#define A0        PF0
#define A1        PF1 
#define A2        PF4 
#define B0        PF5   
#define B1        PF6 
#define D         PF7 
// ROW (must not be changed! Otherwise some functionts wont work)
#define SET_ROW   1
#define RESET_ROW 0
#define DDR_ROW   DDRD
#define PORT_ROW  PORTD
#define MUX_A     PD6
#define MUX_B     PD7 
#define MUX_C     PD2 
#define MUXS_EN   PD3 
#define MUXR_EN   PD4 
// PIR Pins
#define DDR_PIR   DDRB
#define PORT_PIR  PORTB
#define PIN_PIR   PINB
#define OUT_PIR   PB7


// Serial-Buffer
String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete

// Display-Buffer
uint8_t matrix[21][7] = {{0},{0}};

// Pins needed for RTC
int ledPin      = 13;    // LED connected to digital pin 13
int INT_PIN     = 7;     // ext. INT6
int int_number  = 7;

// Variables needed for time stuff
int counter     = 0;
bool addZeroH = false, addZeroM = false, minAgo = false, humanDetected = false, dispCleard = false;
int h,m; 
char hBUFF[10],mBUFF[10],timeBUFF[10];
char *pH, *pM, *pTime; 

// RTC Object
DS1339 RTC = DS1339(INT_PIN, int_number);

void setup() {
  Serial.begin(115200);
  pinMode(INT_PIN, INPUT);
  pinMode(ledPin, OUTPUT);   
  digitalWrite(INT_PIN, HIGH); 
  digitalWrite(ledPin, LOW);
  // ensure RTC oscillator is running, if not already
  RTC.start(); 
  // Set Interrupt alarm every second
  RTC.setAlarmRepeat(EVERY_SECOND); 
  RTC.writeAlarm();

  // default pin-configuration
  DDR_COL = 0xFF;
  PORT_COL = 115;    
  DDR_ROW = 0xFF;
  PORT_ROW = 0b00011000;
  DDR_PIR = 0;

  // Interrupt called after every second to count up to 60
  attachInterrupt(digitalPinToInterrupt(int_number), MinuteCounter, RISING);

  // disp actual time on startup
  DispTime();
}

void loop() { 
  serialEvent();
  //delay(1000);
  //Serial.print("hD ");Serial.println(humanDetected);
  //Serial.print("mA ");Serial.println(minAgo);
  
  // if 60seconds passed...
  if(minAgo == true && humanDetected == true){ 
    DispTime(); 
    dispCleard = false;
    minAgo = false;
  }// else print time if disp was cleard before
  else if(dispCleard==true && humanDetected == true){
    DispTime();
    dispCleard = false;
  }
  // needs here some time (but don't ask why...) 
  delay(1000);
  if(humanDetected == false && dispCleard == false) {
    ClearDisplay();
    RefreshDisplay();
    dispCleard = true;
  }

  // Check for Serial inputs
  if(stringComplete){
    if(inputString == "fill\r"){
        Serial.println("Fill Display");
        FillDisplay();
        RefreshDisplay();
        blink();
     }
     else if(inputString == "clear\r"){
       Serial.println("Clear Display");
       ClearDisplay();
       RefreshDisplay();
       blink();
     }
     else if(inputString == "SET\r"){
       set_time(); 
       blink(); 
     }
     else if(inputString == "READ\r"){
       read_time();  
       blink();
     }
     inputString = "";
     stringComplete = false;
   }  
}

void serialEvent(){
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    inputString += inChar;
    if (inChar == '\n' || inChar == '\r') {
      stringComplete = true;
    }
  }
}

void DispTime(){
      // update RTC library's buffers from chip
      RTC.readTime(); 
      memset(hBUFF,0,sizeof(hBUFF));    
      memset(mBUFF,0,sizeof(mBUFF));      
      // Get Time and convert to Char Array            
      h = int(RTC.getHours());
      m = int(RTC.getMinutes());  
      if(h>=10) addZeroH  = false;
      else addZeroH       = true;
      if(m>=10) addZeroM  = false;  
      else addZeroM       = true;
      itoa(h, hBUFF,10);             
      itoa(m, mBUFF,10);    
      // set pointers to first field in array
      pH = &hBUFF[0]; 
      pM = &mBUFF[0];
      pTime = &timeBUFF[0];
      /* make a Time Array (HH:MM) */
      // add hours
      for(int i=0;i<2;i++){
        if(addZeroH==true){*pTime = '0'; addZeroH=false; pTime++;}       
        else {*pTime = *pH; pTime++; pH++;}
      }
      // add ":" character
      *pTime = ':';
      pTime++;
      // add minutes      
      for(int i=0;i<2;i++){
        if(addZeroM==true){*pTime = '0'; addZeroM=false; pTime++;} 
        else{*pTime = *pM; pTime++; pM++;}
      }    
      // Set Time on Display
      ClearDisplay();
      SetText(timeBUFF);  
      //Serial.print("Time: ");Serial.println(timeBUFF);
      //RefreshSerial();
      RefreshDisplay();
      minAgo = false;
}

void blink(){
  digitalWrite(ledPin, HIGH);
  delay(100);
  digitalWrite(ledPin, LOW); 
}

void MinuteCounter(){  
  // check every second if a humen is detectable
  if(PIN_PIR & (1<<OUT_PIR)) {
    humanDetected = true;
  }
  else {
    humanDetected = false;
  }
  // count up to 1 minute
  counter++;
  if(counter>=60){
    counter=0;
    minAgo=true;      
  }
}

/************************************
 LOW LEVEL FUNCTIONS
************************************/
static const uint8_t LUT_COL[21] = {0,1,2,3,16,17,18,32,33,34,35,48,49,50,64,65,66,67,80,81,82};
/* Set COLUMN 0 to 20, HIGH or LOW. 
 * This function is working just for the Pinmapping on D2-D7
 * To pull an output HIGH on the FP2800, Portpin F7 must be set to 1 otherwise to 0. 
 * Makros: state(SET_HIGH/SET_LOW) */
void Column(uint8_t col, uint8_t state){
  if(col>20) {PORT_COL = 115; return;}
  switch(state){
    case 0 : PORT_COL = 128 | LUT_COL[col]; break;  //( set uC-Pin HIGH -> D-Pin LOW)  
    case 1 : PORT_COL =       LUT_COL[col]; break;  //( set uC-Pin LOW -> set D-Pin HIGH)
    default: PORT_COL = 115;                break;  // default pin-settings 
  }
}

static const uint8_t LUT_ROW[8] = {0,64,128,192,4,68,132,196};
void Row(uint8_t row, uint8_t state){
  // PORTD = D7|D6|X|D4|D3|D2|X|X
  //          B  A    R  S  C  
  if(row>6) {PORT_ROW = 0b00011000; return;}
  switch(state){
    case 0 : PORT_ROW = 8  | LUT_ROW[row];    break;  // reset
    case 1 : PORT_ROW = 16 | LUT_ROW[row];    break;  // set   
    default: PORT_ROW = 0b00011000;           break;  // defaul pin-settings
  }
}
/* Disable both multiplexer to force Coil SET and RESET pins to HI-Z */
void RowOFF(void){
  PORT_ROW = 0b00011000;  
}

/* Disable FP2800 */
void ColOFF(void){
  PORT_COL = 115;   
}

/************************************
 MID LEVEL FUNCTIONS
************************************/
/**
 * Set a pixel in the current working buffer
 */
void SetPixel(uint8_t x, uint8_t y){
  if(x>20 || y>6) return;
  Column(x, SET_LOW);
  Row(y, SET_ROW);
  _delay_us(SWITCH_TIME);
  RowOFF();
  ColOFF();
}
/**
 * Reset a pixel in the current working buffer
 */
void ResetPixel(uint8_t x, uint8_t y){
  if(x>20 || y>6) return;
  Column(x, SET_HIGH);
  Row(y, RESET_ROW);
  _delay_us(SWITCH_TIME);
  RowOFF();
  ColOFF();
}

/************************************
 HIGH LEVEL FUNCTIONS
************************************/
/**
 * Clear display in the current working buffer
 */
void ClearDisplay(){
  for(int i=0;i<21;i++){
    for(int j=0;j<7;j++){
      matrix[i][j] = 0;
    }
  }
}
/**
 * Fill display in the current working buffer
 */
void FillDisplay(){
  for(int i=0;i<21;i++){
    for(int j=0;j<7;j++){
      matrix[i][j] = 1;
    }
  }
}

/**
 * Set a single character
 */
void SetChar(uint8_t x, uint8_t y, unsigned char c){  
  if((x >= 21)                || // Clip right
     (y >= 7)                 || // Clip bottom
     (x>16)                   || // Clip left
     (y>0) )                     // Clip top
    return;
  for(uint8_t i=0;i<6;i++){
    uint8_t line;
    if   (i == 5) line = 0x0;
    else line = pgm_read_byte(font2+(c*5)+i);
    for(uint8_t j=0;j<8;j++){
      if(line & 0x1){
        matrix[x+i][y+j] = 1;
      }
      else{
        matrix[x+i][y+j] = 0;       
      }
      line >>= 1;
    }
  } 
}

/**
 * Print an Textarray
 */
void SetText(const char *s){
  uint8_t x=0;
  while(*s){
    //Serial.println(*s);
    SetChar(x,0,*s++);
    if(*s==':')
      x=x+4;
    else    
      x=x+4;
  }
}

/**
 * Fill Display like a snake
 */
void ClearSnake(int speeed){
  for(int i=0;i<21;i++){
    for(int j=0;j<7;j++){
      SetPixel[i][j];
      _delay_ms(speeed);
    }
  }
  for(int i=0;i<21;i++){
    for(int j=0;j<7;j++){
      ResetPixel[i][j];
      _delay_ms(speeed);
    }
  }
}

/**
 * Invert workingbuffer
 */
void InvertDisplay(){
  for(int i=0;i<21;i++){
    for(int j=0;j<7;j++){
      if(matrix[i][j] == 1){
        matrix[i][j] = 0;
      }
    }
  }   
}

/**
 * Fill display from workingbuffer
 */
void RefreshDisplay(){
  for(int i=0;i<21;i++){
    for(int j=0;j<7;j++){
      if(matrix[i][j] == 1)
        SetPixel(i,j);
      else
        ResetPixel(i,j);     
    }
  }
}


/**
 * Print the workingbuffer an the Serialbus
 */
void RefreshSerial(){
  for(int j=0;j<7;j++){
    for(int i=0;i<21;i++){
      if(matrix[i][j] == 1)
        Serial.print("o");
      else
        Serial.print(".");    
    }
    Serial.println();
  }
}

//************************************
// REAL-TIME-CLOCK (RTC) FUNCTIONS
//************************************
int read_int(char sep)
{
  static byte c;
  static int i;

  i = 0;
  while (1)
  {
    while (!Serial.available())
    {;}
 
    c = Serial.read();
    // Serial.write(c);
  
    if (c == sep)
    {
      // Serial.print("Return value is");
      // Serial.println(i);
      return i;
    }
    if (isdigit(c))
    {
      i = i * 10 + c - '0';
    }
    else
    {
      Serial.print("\r\nERROR: \"");
      Serial.write(c);
      Serial.print("\" is not a digit\r\n");
      return -1;
    }
  }
}

int read_int(int numbytes)
{
  static byte c;
  static int i;
  int num = 0;

  i = 0;
  while (1)
  {
    while (!Serial.available()){;}
 
    c = Serial.read();
    num++;
    // Serial.write(c);
  
    if (isdigit(c))
    {
      i = i * 10 + c - '0';
    }
    else
    {
      Serial.print("\r\nERROR: \"");
      Serial.write(c);
      Serial.print("\" is not a digit\r\n");
      return -1;
    }
    if (num == numbytes)
    {
      // Serial.print("Return value is");
      // Serial.println(i);
      return i;
    }
  }
}

int read_date(int *year, int *month, int *day, int *hour, int* minute, int* second)
{

  *year = read_int(4);
  *month = read_int(2);
  *day = read_int(' ');
  *hour = read_int(':');
  *minute = read_int(':');
  *second = read_int(2);

  return 0;
}

void set_time()
{
    Serial.println("Enter date and time (YYYYMMDD HH:MM:SS)");
    int year, month, day, hour, minute, second;
    int result = read_date(&year, &month, &day, &hour, &minute, &second);
    if (result != 0) {
      Serial.println("Date not in correct format!");
      return;
    } 
    
    // set initially to epoch
    RTC.setSeconds(second);
    RTC.setMinutes(minute);
    RTC.setHours(hour);
    RTC.setDays(day);
    RTC.setMonths(month);
    RTC.setYears(year);
    RTC.writeTime();
    read_time();
}

void read_time() 
{
  Serial.print ("The current time is ");
  RTC.readTime(); // update RTC library's buffers from chip
  printTime(0);
  Serial.println();

}

void printTime(byte type)
{
  // Print a formatted string of the current date and time.
  // If 'type' is non-zero, print as an alarm value (seconds thru DOW/month only)
  // This function assumes the desired time values are already present in the RTC library buffer (e.g. readTime() has been called recently)

  if(!type)
  {
    Serial.print(int(RTC.getMonths()));
    Serial.print("/");  
    Serial.print(int(RTC.getDays()));
    Serial.print("/");  
    Serial.print(int(RTC.getYears()));
  }
  else
  {
    //if(RTC.getDays() == 0) // Day-Of-Week repeating alarm will have DayOfWeek *instead* of date, so print that.
    {
      Serial.print(int(RTC.getDayOfWeek()));
      Serial.print("th day of week, ");
    }
    //else
    {
      Serial.print(int(RTC.getDays()));
      Serial.print("th day of month, ");      
    }
  }
  
  Serial.print("  ");
  Serial.print(int(RTC.getHours()));
  Serial.print(":");
  Serial.print(int(RTC.getMinutes()));
  Serial.print(":");
  Serial.print(int(RTC.getSeconds()));  
}
