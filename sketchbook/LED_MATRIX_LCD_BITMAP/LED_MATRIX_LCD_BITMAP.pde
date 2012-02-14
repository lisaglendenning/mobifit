#include <stdio.h>

/***************************************************************************
* View
***************************************************************************/

#define ROWCOL(r,c,nc) ((r)*(nc)+(c))

// LCD dimensions
#define LCD_XMIN (0)
#define LCD_YMIN (0)
#define LCD_XMAX (83)
#define LCD_YMAX (47)

// LCD serial commands
#define LCD_BPS (9600)
#define LCD_LOCATE ('P')   // set pointer location
#define LCD_CLEAR  ('N')   // clears screen and moves pointer to (0,0)
#define LCD_LINE   ('L')   // draw line from pointer
#define LCD_TONE   ('T')   // audio tone
#define LCD_STRING ('S')   // ASCII text
#define LCD_BITMAP ('B')   // draw bitmap

// LCD command delays in ms
#define LCD_LOCATE_DELAY  (5UL)
#define LCD_CLEAR_DELAY   (70UL)
#define LCD_BITMAP_DELAY  (5UL)
#define LCD_STRING_DELAY  (5UL)
#define LCD_TONE_DELAY    (200UL)

// LCD low beep tone parameters
#define BEEP_LOW_P (100)
#define BEEP_LOW_E (4)
#define BEEP_LOW_T (12)

#define BITMAP_NROWS (6)
#define BITMAP_NCOLS (84)
#define BITMAP_Y_PER_ROW (8)
// BITMAP_Y_PER_ROW must be a power of 2, and
// BITMAP_Y_PER_ROW_SHIFT_MULT is the log of BITMAP_Y_PER_ROW
#define BITMAP_Y_PER_ROW_SHIFT_MULT (3) 
// This is the and mask for doing quick modulus operations
#define BITMAP_Y_PER_ROW_MOD_MASK (0x07) 

byte bitmap[BITMAP_NROWS*BITMAP_NCOLS];


typedef struct _sprite{
  signed char w;
  signed char h;
  signed char x;
  signed char y;
  byte *image;
  volatile byte *tmp;
  boolean draw;
} sprite;

#define CURSOR_NROWS (1)
#define CURSOR_NCOLS (6)

byte cursor_bmp[CURSOR_NROWS*CURSOR_NCOLS] = {
 B00011000, 
 B00111100, 
 B01111110, 
 B11111111, 
 B00011000, 
 B00011000};
  byte cursor_tmp[CURSOR_NCOLS] = {0,0,0,0,0,0};
 //sprite cursorSprite = { CURSOR_NCOLS, CURSOR_NROWS, 48, 24, cursor_bmp, cursor_tmp };
byte cursor2_bmp[CURSOR_NROWS*CURSOR_NCOLS] = {
 B00000000,
 B11111111, 
 B11111111, 
 B00011011, 
 B00001110, 
 B00000100};
  byte cursor2_tmp[CURSOR_NCOLS] = {0,0,0,0,0,0};


volatile sprite spriteList[10]= {
    { CURSOR_NCOLS, CURSOR_NROWS, 48, 24, cursor_bmp, cursor_tmp, true },
    { CURSOR_NCOLS, CURSOR_NROWS, 48, 24, cursor2_bmp, cursor2_tmp, true },
    0, 0, 0, 0, 0, 0, 0, 0};
int oldSpriteX[10];
int oldSpriteY[10];
boolean wasDrawn[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
byte numSprites = 2;

// LED pins
// To turn on an LED, set row pin to LOW and col pin to HIGH
#define LED_NPINS (6)

const int LED_ROW_PINS[] = {
  2, 3, 4, 5, 6, 7};
const int LED_COL_PINS[] = {
  8, 9, 10, 11, 12, 13};

#define LED_TICK 1

#define MATRIX_NROWS (6)
#define MATRIX_NCOLS (6)
#define MATRIX_PLANES (2)
#define NUM_MATRIX_PLANE_CYCLES (7)
byte planeSequence[] = {0,0,1,1,1,1,1};
byte planeCycle = 0;
volatile byte matrixMap[MATRIX_PLANES][MATRIX_NCOLS];
byte currMatrixRow = 0;

int LEDtimerLoadValue = 0xFFFF;
int LCDtimerLoadValue = 0xFFFF;

//void LEDInterruptHandler(){
ISR(TIMER2_OVF_vect) {
//ISR(TIMER1_OVF_vect) {
//unsigned char sreg; 
/* Save global interrupt flag */ 
//sreg = SREG; 
/* Disable interrupts */ 
//cli(); 

  // Update the row and plane for
  // time-multiplexed grey scale
  byte oldRow = currMatrixRow;
  currMatrixRow++;
  if(currMatrixRow==MATRIX_NROWS){
    currMatrixRow=0;
    planeCycle++;
    if(planeCycle >= NUM_MATRIX_PLANE_CYCLES) 
      planeCycle = 0;
  }
  byte plane = planeSequence[planeCycle];

  // Merge new output with existing values
  byte newOutput = (PORTB & B11000000) | (matrixMap[plane][currMatrixRow] & B00111111);

  // Display the next column
  byte maskHIGH, maskLOW;
  maskHIGH = 0x01 << LED_ROW_PINS[oldRow];
  maskLOW = ~(0x01 << LED_ROW_PINS[currMatrixRow]);

  PORTD |=maskHIGH;
  PORTB = newOutput;
  PORTD &= maskLOW;

  //Capture the current timer value. This is how much error we
  //have due to interrupt latency and the work in this function
  byte latency=TCNT2;

  //Reload the timer and correct for latency.
  TCNT2=latency+LEDtimerLoadValue;
  
    //Capture the current timer value. This is how much error we
  //have due to interrupt latency and the work in this function
  //int latency=TCNT1;

  //Reload the timer and correct for latency.
  //TCNT1=latency+LCDtimerLoadValue;

//SREG = sreg;
//interrupts();
}
/***************************************************************************
***************************************************************************/

byte byte_layer(byte bg, byte fg, boolean add) {
  byte b = bg;
  if (add) {
    b |= fg;
  } else {
    b &= ~fg;
  }

  return b;
}

/***************************************************************************
***************************************************************************/

//void bitmap_layer(byte *bg, byte bg_nrows, byte bg_ncols, byte *fg,
//  byte fg_nrows, byte fg_ncols, int xoffset, int yoffset, boolean add) {

//void LCDInterruptHandler(){
ISR(TIMER1_OVF_vect, ISR_NOBLOCK) {
  //noInterrupts();

  byte *bg = bitmap;
  byte bg_nrows = BITMAP_NROWS;
  byte bg_ncols = BITMAP_NCOLS;
  for(int spt=0; spt<numSprites; spt++){
    signed char x = spriteList[spt].x;
    signed char y = spriteList[spt].y;
   
    if(wasDrawn[spt] && (x != oldSpriteX[spt] || y != oldSpriteY[spt])){
      // We need to erase the old sprite
      signed char xstart = oldSpriteX[spt];
      signed char xend = xstart + spriteList[spt].w;
      signed char ystart = oldSpriteY[spt] ;
      signed char yend = ystart + (spriteList[spt].h<<BITMAP_Y_PER_ROW_SHIFT_MULT);
      byte* fg = spriteList[spt].image;
      
      signed char bg_row_start;
      signed char bg_row_end;
      signed char bg_y_shift;
      signed char fg_row_offset;
      
      signed char col;
      // enforce X boundaries
      signed char colMin = (xstart<0?-xstart:0);
      signed char bg_col_start = xstart;//(xstart<0?0:xstart);
      
      signed char colMax = (xend > BITMAP_NCOLS ? BITMAP_NCOLS : xend);
      colMax -= xstart;
      
      // enforce Y boundaries
      if (ystart < 0) {
	bg_y_shift = BITMAP_Y_PER_ROW - (-ystart & BITMAP_Y_PER_ROW_MOD_MASK);
	fg_row_offset = (BITMAP_Y_PER_ROW-1-ystart)>>BITMAP_Y_PER_ROW_SHIFT_MULT;
	bg_row_start = 0;
      } else {
	bg_y_shift = ystart  & BITMAP_Y_PER_ROW_MOD_MASK;
	fg_row_offset = 0;
	bg_row_start = ystart>>BITMAP_Y_PER_ROW_SHIFT_MULT;
      }
      if (yend > bg_nrows<<BITMAP_Y_PER_ROW_SHIFT_MULT) {
	bg_row_end = bg_nrows;
      } else {
	bg_row_end = (BITMAP_Y_PER_ROW-1+yend)>>BITMAP_Y_PER_ROW_SHIFT_MULT;
      }

      volatile byte* scratch = spriteList[spt].tmp;
      for (byte row=0;  bg_row_start+row<bg_row_end;  ++row) {
	Serial.print(LCD_LOCATE, BYTE);
	Serial.print(bg_col_start+colMin, BYTE);
	Serial.print((bg_row_start+row)<<BITMAP_Y_PER_ROW_SHIFT_MULT, BYTE);
	
        int bgIdx = ROWCOL(bg_row_start+row,bg_col_start+colMin,bg_ncols);
        for (col=colMin; col<colMax;  ++col,++bgIdx) {
	  byte b = bg[bgIdx];
          scratch[col] = b;
        }
	
        Serial.print(LCD_BITMAP, BYTE);
	Serial.print(colMax-colMin, BYTE);
        for (col=colMin; col<colMax;  ++col) {
  	  Serial.print(scratch[col], BYTE);
	}
      }
 
    }
  }
  for(int spt=0; spt<numSprites; spt++){
    signed char x = spriteList[spt].x;
    signed char y = spriteList[spt].y;
    signed char w = spriteList[spt].w;
    signed char h = spriteList[spt].h;
    if(spriteList[spt].draw){
      oldSpriteX[spt]=x;
      oldSpriteY[spt]=y;
      wasDrawn[spt]=true;
      signed char xstart = x;
      signed char xend = x + w;
      signed char ystart = y ;
      signed char yend = y + h*BITMAP_Y_PER_ROW;
      byte* fg = spriteList[spt].image;
      
      signed char bg_col_start;
      signed char bg_col_end;
      signed char fg_col_offset;
      signed char bg_row_start;
      signed char bg_row_end;
      signed char fg_row_offset;
      signed char bg_y_shift;
      
      // enforce X boundaries
      if (xstart < 0) {
	fg_col_offset = -xstart;
	bg_col_start = 0;
      } else {
	fg_col_offset = 0;
	bg_col_start = xstart;
      }
      if (xend > bg_ncols) {
	bg_col_end = bg_ncols;
      } else {
	bg_col_end = xend;
      }
      
      // enforce Y boundaries
      if (ystart < 0) {
	bg_y_shift = BITMAP_Y_PER_ROW - (-ystart & BITMAP_Y_PER_ROW_MOD_MASK);
	fg_row_offset = (BITMAP_Y_PER_ROW-1-ystart)>>BITMAP_Y_PER_ROW_SHIFT_MULT;
	bg_row_start = 0;
      } else {
	bg_y_shift = ystart  & BITMAP_Y_PER_ROW_MOD_MASK;
	fg_row_offset = 0;
	bg_row_start = ystart>>BITMAP_Y_PER_ROW_SHIFT_MULT;
      }
      if (yend > bg_nrows<<BITMAP_Y_PER_ROW_SHIFT_MULT) {
	bg_row_end = bg_nrows;
      } else {
	bg_row_end = (BITMAP_Y_PER_ROW-1+yend)>>BITMAP_Y_PER_ROW_SHIFT_MULT;
      }
      volatile byte* scratch = spriteList[spt].tmp;
      for (byte row=0;  bg_row_start+row<bg_row_end;  ++row) {
	  Serial.print(LCD_LOCATE, BYTE);
	  Serial.print(bg_col_start, BYTE);
	  Serial.print((bg_row_start+row)*BITMAP_Y_PER_ROW, BYTE);
//	  delay(LCD_LOCATE_DELAY);
        int bgIdx = ROWCOL(bg_row_start+row,bg_col_start,bg_ncols);
        int fgIdxL = ROWCOL(fg_row_offset+row-1,fg_col_offset,w);
        int fgIdxH = ROWCOL(fg_row_offset+row,fg_col_offset,w);
	for (byte col=0;  bg_col_start+col<bg_col_end;  ++col, ++bgIdx, ++fgIdxL, ++fgIdxH) {
	  byte b = bg[bgIdx];
	  if ((row > 0) || (ystart < 0)) {
	    byte fg_prev = fg[fgIdxL]
	      >> (BITMAP_Y_PER_ROW - bg_y_shift);
            
	    b = b|fg_prev;  //byte_layer(b, fg_prev, true);
	  }
	  
	  if ((bg_row_start+row+1<bg_row_end) || (yend > bg_nrows*BITMAP_Y_PER_ROW) || (bg_y_shift == 0)) {
	    byte fg_next = fg[fgIdxH] << bg_y_shift;
	    b = b|fg_next;//byte_layer(b, fg_next, true);
	  }
          scratch[col] = b;	  
        }
	  Serial.print(LCD_BITMAP, BYTE);
	  Serial.print(bg_col_end-bg_col_start, BYTE);
        for (byte col=0;  bg_col_start+col<bg_col_end;  ++col) {
  	  Serial.print(scratch[col], BYTE);
	}
//	  delay(LCD_BITMAP_DELAY);
      }
    } else{
      wasDrawn[spt]=false;
    }
  } 




  //Capture the current timer value. This is how much error we
  //have due to interrupt latency and the work in this function
  int latency=TCNT1;

  //Reload the timer and correct for latency.
  TCNT1=latency+LCDtimerLoadValue;
 
  //interrupts();
}

#define LCD_TIMER_CLOCK_FREQ  32150.0 //1MHz for /256 prescale from 8MHz
#define LED_TIMER_CLOCK_FREQ  32150.0 //32kHz for /256 prescale from 8MHz

//Setup Timer1.
//Configures the ATMega168 8-Bit Timer2 to generate an interrupt
//at the specified frequency.
//Returns the timer load value which must be loaded into TCNT2
//inside your ISR routine.
//See the example usage below.
unsigned char SetupTimer1(float timeoutFrequency){
  unsigned char result; //The timer load value.

  //Calculate the timer load value
  result=(int)((65536.0-(LCD_TIMER_CLOCK_FREQ/timeoutFrequency))+0.5);

  //Timer2 Settings: Timer Prescaler /8, mode 0
  //Timer clock = 16MHz/8 = 2Mhz or 0.5us
  //The /8 prescale gives us a good range to work with
  //so we just hard code this for now.
  //CS12 CS11 CS10 Description 
  //0    0    0    No clock source (Timer/Counter stopped). 
  //0    0    1    clkI/O/1 (No prescaling) 
  //0    1    0    clkI/O/8 (From prescaler) 
  //0    1    1    clkI/O/64 (From prescaler) 
  //1    0    0    clkI/O/256 (From prescaler) 
  //1    0    1    clkI/O/1024 (From prescaler) 
  //1    1    0    External clock source on T1 pin. Clock on falling edge. 
  //1    1    1    External clock source on T1 pin. Clock on rising edge. 

  TCCR1A = 0;
  TCCR1B = 0<<CS12 | 1<<CS11 | 1<<CS10;

  //Timer2 Overflow Interrupt Enable
  TIMSK1 = 1<<TOIE1;

  //load the timer for its first cycle
  TCNT1=result;

  return(result);
}

//Setup Timer2.
//Configures the ATMega168 8-Bit Timer2 to generate an interrupt
//at the specified frequency.
//Returns the timer load value which must be loaded into TCNT2
//inside your ISR routine.
//See the example usage below.
unsigned char SetupTimer2(float timeoutFrequency){
  unsigned char result; //The timer load value.

  //Calculate the timer load value
  result=(int)((256.0-(LED_TIMER_CLOCK_FREQ/timeoutFrequency))+0.5);

  //Timer2 Settings: Timer Prescaler /8, mode 0
  //Timer clock = 16MHz/8 = 2Mhz or 0.5us
  //The /8 prescale gives us a good range to work with
  //so we just hard code this for now.
  //CS12 CS11 CS10 Description 
  //0    0    0    No clock source (Timer/Counter stopped). 
  //0    0    1    clkI/O/1 (No prescaling) 
  //0    1    0    clkI/O/8 (From prescaler) 
  //0    1    1    clkI/O/64 (From prescaler) 
  //1    0    0    clkI/O/256 (From prescaler) 
  //1    0    1    clkI/O/1024 (From prescaler) 
  //1    1    0    External clock source on T1 pin. Clock on falling edge. 
  //1    1    1    External clock source on T1 pin. Clock on rising edge. 
  TCCR2A = 0;
  TCCR2B = 1<<CS22 | 0<<CS21 | 0<<CS20;

  //Timer2 Overflow Interrupt Enable
  TIMSK2 = 1<<TOIE2;

  //load the timer for its first cycle
  TCNT2=result;

  return(result);
}

void setup(){
  
  // This must be here so we can program the device
  delay(1000);
  
  
  for (byte i=0;  i<LED_NPINS;  ++i) {
    pinMode(LED_ROW_PINS[i], OUTPUT);
    digitalWrite(LED_ROW_PINS[i], HIGH);
    pinMode(LED_COL_PINS[i], OUTPUT);
    digitalWrite(LED_COL_PINS[i], LOW);
  }

  matrixMap[0][0] = B00111111;
  matrixMap[0][1] = B00111110;
  matrixMap[0][2] = B00111100;
  matrixMap[0][3] = B00111000;
  matrixMap[0][4] = B00110000;
  matrixMap[0][5] = B00100000;
  
  matrixMap[1][0] = B00101010;
  matrixMap[1][1] = B01010101;
  matrixMap[1][2] = B00101010;
  matrixMap[1][3] = B01010101;
  matrixMap[1][4] = B00101010;
  matrixMap[1][5] = B01010101;
  
  byte row;
  byte col;
  for (row=0;  row<BITMAP_NROWS;  ++row) {
    for (col=0;  col<BITMAP_NCOLS;  ++col) {
      bitmap[ROWCOL(row,col,BITMAP_NCOLS)] = B00000000;
    }
  }


  LCDtimerLoadValue = SetupTimer1(2.0f);
  
  // 720Hz update is 30fps for the whole lcd display
  LEDtimerLoadValue = SetupTimer2(720.0f);
  
  spriteList[0].draw = true;
Serial.begin(LCD_BPS);

Serial.print(LCD_CLEAR, BYTE);
delay(LCD_CLEAR_DELAY);

char strbuf[16];
int n = snprintf(strbuf, 15, "Display Test !\n");
Serial.print(LCD_STRING, BYTE);
Serial.print(n, BYTE);
for (byte i=0;  i<n;  ++i) {
  Serial.print(strbuf[i], BYTE);
}

}



int dir = 1;  
void loop(){
  //LCDInterruptHandler();
  //LEDInterruptHandler();
  for(int i=0; i<MATRIX_NCOLS; i++){
      if(i%2==dir){
        matrixMap[0][i] = (matrixMap[0][i]>>2)|(matrixMap[0][i]<<6); 
      } else {
        matrixMap[0][i] = (matrixMap[0][i]<<2)|(matrixMap[0][i]>>6); 
      }
    }      
    if(dir == 1)
      dir = 0; 
    else
      dir = 1;    

//    if(i%2==0){
//      matrixMap[0][i] = (matrixMap[0][i]>>1)|(matrixMap[0][i]<<7); 
//    } else {
//      matrixMap[0][i] = (matrixMap[0][i]<<1)|(matrixMap[0][i]>>7); 
//    }
    spriteList[0].x+=1;  
    if(spriteList[0].x>84) spriteList[0].x = 0;
    
    spriteList[0].y+=1;   
    if(spriteList[0].y>48) spriteList[0].y = 0;
    
    
    spriteList[1].x-=2;  
    if(spriteList[1].x<0) spriteList[1].x = 84;
    
    spriteList[1].y-=1;   
    if(spriteList[1].y<0) spriteList[1].y = 48;
//  }


  delay(200);  
}

