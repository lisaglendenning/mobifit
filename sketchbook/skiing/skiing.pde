/***************************************************************************
 * Description:    FSR Skiing Balance Game
 * Authors:        Lisa Glendenning, Stephen Friedman
 * Last Modified:  March 2009
 ***************************************************************************/
#include <stdio.h>

#define DEBUG (0)
#define LOOP_DELAY (12UL)  // ms

/***************************************************************************
 * Control
 ***************************************************************************/

// Range of raw sensor input
#define FSR_MIN (0)
#define FSR_MAX (1023)

// Number of FSR sensors
#define FSR_N (4)

// array indices for left/right and toe/heel
#define FSR_LT (0)
#define FSR_LH (1)
#define FSR_RT (2)
#define FSR_RH (3)

// FSR input pins
#define FSR_PIN_LT (3)
#define FSR_PIN_LH (2)
#define FSR_PIN_RT (1)
#define FSR_PIN_RH (0)

// Sample size for averaging filter
#define FSR_SAMPLE_N  (5)

// Time used for calibration
#define CALIBRATE_MS (1000UL)

// current value of FSRs
int fsrs[] = {
  0, 0, 0, 0};

// running average value of FSRs
int fsrs_avg[] = {
  0, 0, 0, 0};

// calibrated "zero" of FSRs
int fsrs_zero[] = {
  0, 0, 0, 0};

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

// Bitmap dimensions (bottom row is reserved for text)
#define BITMAP_NROWS (5)
#define BITMAP_NCOLS (84)
#define BITMAP_XMIN (0)
#define BITMAP_YMIN (0)
#define BITMAP_XMAX (83)
#define BITMAP_YMAX (40)
#define BITMAP_Y_PER_ROW (8)
// BITMAP_Y_PER_ROW must be a power of 2, and
// BITMAP_Y_PER_ROW_SHIFT_MULT is the log of BITMAP_Y_PER_ROW
#define BITMAP_Y_PER_ROW_SHIFT_MULT (3) 
// This is the and mask for doing quick modulus operations
#define BITMAP_Y_PER_ROW_MOD_MASK (0x07) 

// bitmap dimensions
#define FLAG_NROWS (1)
#define FLAG_NCOLS (4)

// Constant x/y spacing between flags
#define FLAGSET_DX (30)
#define FLAGSET_DY (25)

// Range that valid x of a left flag can be in
#define LFLAG_XMIN (0)
#define LFLAG_XMAX (LCD_XMAX - FLAG_NCOLS - FLAGSET_DX + 1)



byte lflag_bmp[FLAG_NROWS*FLAG_NCOLS] = {
 B00000100, 
 B00001110, 
 B00011011, 
 B11111111, 
};

byte rflag_bmp[FLAG_NROWS*FLAG_NCOLS] = {
 B11111111, 
 B00011011, 
 B00001110, 
 B00000100
};

// As long as LFLAG and RFLAG are the same size,
// they can use the same temporary memory.
byte flag_tmp[FLAG_NROWS*FLAG_NCOLS] = {0,0,0,0};

#define SKIER_NROWS (2)
#define SKIER_NCOLS (7)

byte skier_bmp[SKIER_NROWS*SKIER_NCOLS] = {
 B00000000, 
 B11100000, 
 B11110111, 
 B11111111, 
 B11110111, 
 B11100000, 
 B00000000, 

 B11111110, 
 B00011000, 
 B00001111, 
 B00000111, 
 B00001111, 
 B00011000, 
 B11111110 
};
byte skier_tmp[SKIER_NROWS*SKIER_NCOLS] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0};
  

typedef struct _sprite{
  signed char w;
  signed char h;
  signed char x;
  signed char y;
  byte *image;
  volatile byte *tmp;
  boolean draw;
} sprite;


#define NSPRITES (9)
#define SKIER_SPRITE (8)

// sprite x,y is the upper left position
volatile sprite spriteList[NSPRITES];
int oldSpriteX[NSPRITES];
int oldSpriteY[NSPRITES];
boolean wasDrawn[NSPRITES] = {0, 0, 0, 0, 0, 0, 0, 0, 0};

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

unsigned char LEDtimerLoadValue = 0xFFFF;
unsigned int LCDtimerLoadValue = 0xFFFF;

#define TEXT_X (16)
#define TEXT_Y (40)
#define TEXT_LEN (9)

char text_buf[TEXT_LEN];
char debug_buf[15];
int debug_buf_len = 0;


boolean update_text_flag = 1;
boolean display_text_flag = 1;

/***************************************************************************
 * Model
 ***************************************************************************/

#define MODEL_N  (-1)
#define MODEL_E  (1)
#define MODEL_S  (1)
#define MODEL_W  (-1)

#define MODEL_X  (0)
#define MODEL_Y  (1)

#define FLAGSET_XSHIFT (15)

#define SKIER_XSTART (BITMAP_XMIN + BITMAP_NCOLS/2 - SKIER_NCOLS/2 - 1)
#define SKIER_YSTART (BITMAP_YMIN + (BITMAP_NROWS - SKIER_NROWS)*BITMAP_Y_PER_ROW)

#define TIME_LIMIT (100)

#define SCORE_HIT  (2)
#define SCORE_MISS (1)

#define SKIER_YTHRESH (SKIER_YSTART + BITMAP_Y_PER_ROW)
#define NFLAG_SETS (4)

#define DX_LEN (4)
#define DY_LEN (5)

int dx_thresholds[] = { 25, 50, 75, 100 };
int dy_thresholds[] = { -50, -25, 25, 50, 75 };

#define DY_OFFSET (2)

typedef struct ski_model_t {
  volatile int skier_dx;
  volatile int skier_dy;
  byte next_flagset;
  int timeout;
  int score;
} ski_model_t;

unsigned long start_millis = 0;

#define INTRO (1)
#define RUN (2)
#define OUTRO (3)
byte mode = INTRO;

ski_model_t model;

/***************************************************************************
 * Timer ISR Routines
 ***************************************************************************/
ISR(TIMER2_OVF_vect) {
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
  
//SREG = sreg;
//interrupts();
}

/***************************************************************************
***************************************************************************/

//void LCDInterruptHandler(){
ISR(TIMER1_OVF_vect, ISR_NOBLOCK) {
  
  if (DEBUG) { 
    serialWrite(LCD_LOCATE);
    serialWrite(0);
    serialWrite(0);
    
    serialWrite(LCD_STRING);
    serialWrite(debug_buf_len);
    for (byte c=0;  c<debug_buf_len;  c++) {
      serialWrite(debug_buf[c]);
    }  
  } else {
  byte bg_nrows = BITMAP_NROWS;
  byte bg_ncols = BITMAP_NCOLS;
  for(int spt=0; spt<NSPRITES; spt++){
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
	serialWrite(LCD_LOCATE);
	serialWrite(bg_col_start+colMin);
	serialWrite((bg_row_start+row)<<BITMAP_Y_PER_ROW_SHIFT_MULT);
	
        int bgIdx = ROWCOL(bg_row_start+row,bg_col_start+colMin,bg_ncols);
        for (col=colMin; col<colMax;  ++col,++bgIdx) {
	  byte b = 0;//bg[bgIdx];
          scratch[col] = b;
        }
	
        serialWrite(LCD_BITMAP);
	serialWrite(colMax-colMin);
        for (col=colMin; col<colMax;  ++col) {
  	  serialWrite(scratch[col]);
	}
      }
 
    }
  }
  for(int spt=0; spt<NSPRITES; spt++){
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
	  serialWrite(LCD_LOCATE);
	  serialWrite(bg_col_start);
	  serialWrite((bg_row_start+row)*BITMAP_Y_PER_ROW);
//	  delay(LCD_LOCATE_DELAY);
        int bgIdx = ROWCOL(bg_row_start+row,bg_col_start,bg_ncols);
        int fgIdxL = ROWCOL(fg_row_offset+row-1,fg_col_offset,w);
        int fgIdxH = ROWCOL(fg_row_offset+row,fg_col_offset,w);
	for (byte col=0;  bg_col_start+col<bg_col_end;  ++col, ++bgIdx, ++fgIdxL, ++fgIdxH) {
	  byte b = 0;//bg[bgIdx];
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
	  serialWrite(LCD_BITMAP);
	  serialWrite(bg_col_end-bg_col_start);
        for (byte col=0;  bg_col_start+col<bg_col_end;  ++col) {
  	  serialWrite(scratch[col]);
	}
//	  delay(LCD_BITMAP_DELAY);
      }
    } else{
      wasDrawn[spt]=false;
    }
  } 

  if (display_text_flag) {
    display_text_flag = false;

    serialWrite(LCD_LOCATE);
    serialWrite(TEXT_X);
    serialWrite(TEXT_Y);
    
    serialWrite(LCD_STRING);
    serialWrite(TEXT_LEN);
    for(byte c=0; c<TEXT_LEN; c++){
      serialWrite(text_buf[c]);
    }
  }
  }


  //Capture the current timer value. This is how much error we
  //have due to interrupt latency and the work in this function
  unsigned int latency=TCNT1;

  //Reload the timer and correct for latency.
  TCNT1=LCDtimerLoadValue;
 
  //interrupts();
}

/***************************************************************************
 ***************************************************************************/

#define LCD_TIMER_CLOCK_FREQ  32150.0 //1MHz for /256 prescale from 8MHz
#define LED_TIMER_CLOCK_FREQ  32150.0 //32kHz for /256 prescale from 8MHz

//Setup Timer1.
//Configures the ATMega168 8-Bit Timer2 to generate an interrupt
//at the specified frequency.
//Returns the timer load value which must be loaded into TCNT2
//inside your ISR routine.
//See the example usage below.
unsigned int SetupTimer1(float timeoutFrequency){
  unsigned char result; //The timer load value.

  //Calculate the timer load value
  result=(unsigned int)((65536.0-(LCD_TIMER_CLOCK_FREQ/timeoutFrequency))+0.5);

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
  TCCR1B = 1<<CS12 | 0<<CS11 | 0<<CS10;

  //load the timer for its first cycle
  TCNT1=result;

  return(result);
}

void StartTimer1(){
   //Timer2 Overflow Interrupt Enable
  TIMSK1 = 1<<TOIE1;
}

void StopTimer1(){
   //Disable all Timer1 interrupts
  TIMSK1 = 0;
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

  //load the timer for its first cycle
  TCNT2=result;

  return(result);
}

void StartTimer2(){
  //Timer2 Overflow Interrupt Enable
  TIMSK2 = 1<<TOIE2;
}
void StopTimer2(){
   //Disable all Timer2 interrupts
  TIMSK2 = 0;
}


void setupTimers() {
  LCDtimerLoadValue = SetupTimer1(10.0f);
  LCDtimerLoadValue = 0xFF80;
  TCNT1=LCDtimerLoadValue;

  // 720Hz update is 30fps for the whole lcd display
  LEDtimerLoadValue = SetupTimer2(20.0f);  
}

void startTimers(){
  StartTimer1();
  StartTimer2();
}

void stopTimers(){
  StopTimer1();
  StopTimer2();
}

/***************************************************************************
 ***************************************************************************/

void setupControl() {
  pinMode(FSR_PIN_LT, INPUT);
  pinMode(FSR_PIN_LH, INPUT);
  pinMode(FSR_PIN_RT, INPUT);
  pinMode(FSR_PIN_RH, INPUT);
  
  for (byte i=0;  i<LED_NPINS;  ++i) {
    pinMode(LED_ROW_PINS[i], OUTPUT);
    digitalWrite(LED_ROW_PINS[i], HIGH);
    pinMode(LED_COL_PINS[i], OUTPUT);
    digitalWrite(LED_COL_PINS[i], LOW);
  }
  
  Serial.begin(LCD_BPS);  
}

/***************************************************************************
 ***************************************************************************/

void pollFSRs() {
  // Read sensor value in the range [0,1023]
  // where a lower number is higher pressure
  fsrs[FSR_LT] = analogRead(FSR_PIN_LT);
  fsrs[FSR_LH] = analogRead(FSR_PIN_LH);
  fsrs[FSR_RT] = analogRead(FSR_PIN_RT);
  fsrs[FSR_RH] = analogRead(FSR_PIN_RH);

  // Flip values
  for (byte i=0;  i<FSR_N;  ++i) {
    fsrs[i] = abs(fsrs[i]-FSR_MAX);
  }

  // Filter
  for (byte i=0;  i<FSR_N;  ++i) {
    fsrs[i] = ((FSR_SAMPLE_N-1)*fsrs_avg[i] + fsrs[i]) / FSR_SAMPLE_N;
  }
}

/***************************************************************************
 ***************************************************************************/

void setupModel() {
  model.next_flagset = 0;
  model.timeout = TIME_LIMIT;
  model.skier_dx = 0;
  model.skier_dy = 0;
  model.score = 0;
}

/***************************************************************************
 ***************************************************************************/

void updateModel() {
  // Update the time
  long walltime = TIME_LIMIT - ((millis() - start_millis) >> 10);
  if (walltime != model.timeout) {
    model.timeout = walltime;  
    if(model.timeout==0){
      mode = OUTRO;
    }
    update_text_flag = true;
  }  
  
  // Subtract "zero" sensor values
  for(byte i=0;  i<FSR_N;  ++i) {
    fsrs[i] -= fsrs_zero[i];
  }

  // Each FSR reading corresponds to the magnitude A of a 2D vector
  // The directions, d, of the vectors are: LT=(-1,-1), LH=(-1,1),
  // RT=(1,-1), RH=(1,1)
  // Note: For a unit circle we would use sqrt(2)/2 instead of 1
  // We can represent each vector as a*d, where a = sqrt(A2 / 2)
  // Sum the vectors to determine the cartesian velocity

  int a[FSR_N];
  for (byte i=0;  i<FSR_N;  ++i) {
    a[i] = sqrt(sq(long(fsrs[i])) / 2L);
    if (fsrs[i] < 0) {
      a[i] = -a[i];
    }
  }

  int dx = MODEL_W*(a[FSR_LT] + a[FSR_LH]) +
    MODEL_E*(a[FSR_RT] + a[FSR_RH]);
  int dy = -(MODEL_N*(a[FSR_LT] + a[FSR_RT]) +
    MODEL_S*(a[FSR_LH] + a[FSR_RH]));

  // map velocity to a sensible range
  model.skier_dx = 0;
  for (int i=DX_LEN-1;  i>=0;  --i) {
    if (abs(dx) > dx_thresholds[i]) {
      model.skier_dx = (dx < 0) ? -i : i;
      break;
    }  
  }
  model.skier_dy = DY_OFFSET;
  for (int i=DY_LEN-1;  i>=0;  --i) {
    if (dy > dy_thresholds[i]) {
      model.skier_dy = i+DY_OFFSET+1;
      break;
    }  
  }

  // Calculate whether we've hit or missed a flag set
  byte l_flag = model.next_flagset*2;
  byte r_flag = l_flag + 1;
  if (spriteList[l_flag].y+FLAG_NROWS*BITMAP_Y_PER_ROW >= SKIER_YTHRESH) {
    byte xloc = spriteList[SKIER_SPRITE].x - ((l_flag && 0x02) ? 0:3);
    if (spriteList[l_flag].x < xloc &&
        spriteList[r_flag].x > xloc+SKIER_NCOLS) {
      model.score += SCORE_HIT;
    } else if (model.score > 0) {
      model.score -= SCORE_MISS;
    }
    
    if (model.next_flagset == NFLAG_SETS - 1) {
      model.next_flagset = 0;
    } else {
      model.next_flagset++;
    }
    
    update_text_flag = true;
  }
}


/***************************************************************************
 ***************************************************************************/

void updateView() {
  spriteList[SKIER_SPRITE].x = constrain(spriteList[SKIER_SPRITE].x+model.skier_dx,
    BITMAP_XMIN, BITMAP_XMAX-SKIER_NCOLS);
  
  for (byte i=0;  i<NFLAG_SETS;  ++i) {
    byte lflag = 2*i;
    byte rflag = lflag + 1;
    
    spriteList[lflag].y += model.skier_dy;   
    
    // recycle flag once it's moved off the screen  
    if (spriteList[lflag].y > BITMAP_YMAX) {
      spriteList[lflag].y -= FLAGSET_DY*NFLAG_SETS;
      byte prev_flag = (i == 0) ? (NFLAG_SETS-1) : (i-1);
      signed char flag_move = random(2*FLAGSET_XSHIFT);
      signed char newx = spriteList[2*prev_flag].x + flag_move - FLAGSET_XSHIFT;
      newx = constrain(newx, LFLAG_XMIN, LFLAG_XMAX);
      spriteList[lflag].x = newx;
      spriteList[rflag].x = spriteList[lflag].x + FLAGSET_DX;
    }  
    
    spriteList[rflag].y = spriteList[lflag].y; 
  }
  
  
  byte disp_dy = model.skier_dy;
  byte disp_dx = (model.skier_dx);
  byte circle = B01111; 
  byte spot = B00110;
  if(disp_dy>(DY_OFFSET+1)){
    circle <<= (disp_dy-(DY_OFFSET+1));
    spot   <<= (disp_dy-(DY_OFFSET+1));
  }else{
    circle >>= ((DY_OFFSET+1)-disp_dy);
    spot   >>= ((DY_OFFSET+1)-disp_dy);
  }
     for(byte x=0; x < MATRIX_NCOLS; x++){
      byte xoffset = x+8-disp_dx;
      if( xoffset > 0+8 && xoffset < 5+8)
         if( xoffset > 1+8 && xoffset < 4+8 )
            matrixMap[0][x] = circle;
         else
            matrixMap[0][x] = spot;
      else  
        matrixMap[0][x] = 0;
      
      if( xoffset > 1+8 && xoffset < 4+8)
        matrixMap[1][x] = spot;
      else  
        matrixMap[1][x] = 0;
    } 
    
  if (update_text_flag) {
    // Update the timer and score display
    // 01234567890123
    // TXXX SXXXX
    snprintf(text_buf, TEXT_LEN+1, "T%3.3d S%3.3d", model.timeout, model.score);  
    update_text_flag = false;
    display_text_flag = true;
  }
  
  if (DEBUG) {
    debug_buf_len = snprintf(debug_buf, 15, "%4d, %4d", 
      model.skier_dx, model.skier_dy); 
  }
}

/***************************************************************************
 ***************************************************************************/

void setupView() {
  spriteList[SKIER_SPRITE].w = SKIER_NCOLS;
  spriteList[SKIER_SPRITE].h = SKIER_NROWS;
  spriteList[SKIER_SPRITE].x = SKIER_XSTART;
  spriteList[SKIER_SPRITE].y = SKIER_YSTART;
  spriteList[SKIER_SPRITE].image = skier_bmp;
  spriteList[SKIER_SPRITE].tmp = skier_tmp;
  spriteList[SKIER_SPRITE].draw = true;
  
  for (byte i=0;  i<NFLAG_SETS;  ++i) {
    byte lflag = 2*i;
    byte rflag = lflag + 1;
    
    spriteList[lflag].w = FLAG_NCOLS;
    spriteList[lflag].h = FLAG_NROWS;
    spriteList[lflag].image = (i%2 == 0) ? lflag_bmp : rflag_bmp;
    spriteList[lflag].tmp = flag_tmp;
    spriteList[lflag].draw = true;
    
    if (i == 0) {
      spriteList[lflag].y = 0;
      spriteList[lflag].x = BITMAP_NCOLS/2 - (FLAGSET_DX+FLAG_NCOLS)/2;  
    } else {
      spriteList[lflag].y = spriteList[lflag-2].y - FLAGSET_DY;
      spriteList[lflag].x = constrain(spriteList[lflag-2].x + random(-FLAGSET_XSHIFT, FLAGSET_XSHIFT),
        LFLAG_XMIN, LFLAG_XMAX);
    }
    
    spriteList[rflag].w = spriteList[lflag].w;
    spriteList[rflag].h = spriteList[lflag].h;
    spriteList[rflag].image = spriteList[lflag].image;
    spriteList[rflag].tmp = spriteList[lflag].tmp;
    spriteList[rflag].draw = spriteList[lflag].draw;
    spriteList[rflag].y = spriteList[lflag].y;  
    spriteList[rflag].x = spriteList[lflag].x + FLAGSET_DX; 
  }
  clearLEDs();
  update_text_flag=1;
  display_text_flag=1;
}

void clearLEDs(){
     for(byte x=0; x < MATRIX_NCOLS; x++){
        matrixMap[0][x] = 0;
        matrixMap[1][x] = 0;
    } 
}

/***************************************************************************
 ***************************************************************************/

void intro() {
  Serial.print(LCD_CLEAR, BYTE);
  delay(LCD_CLEAR_DELAY);
  
  serialWrite('C');
  serialWrite(70);
  
  Serial.print(LCD_LOCATE, BYTE);
  Serial.print(16, BYTE);
  Serial.print(8, BYTE);

  char strbuf[16];
  int n = snprintf(strbuf, 9, "MobiSki!");
  Serial.print(LCD_STRING, BYTE);
  Serial.print(n, BYTE);
  for (byte i=0;  i<n;  ++i) {
    Serial.print(strbuf[i], BYTE);
  }
  delay(500UL); 
  
  Serial.print(LCD_LOCATE, BYTE);
  Serial.print(24, BYTE);
  Serial.print(24, BYTE);

   n = snprintf(strbuf, 7, "Ready!");
  Serial.print(LCD_STRING, BYTE);
  Serial.print(n, BYTE);
  for (byte i=0;  i<n;  ++i) {
    Serial.print(strbuf[i], BYTE);
  }
  Serial.print(LCD_TONE, BYTE);
  Serial.print(BEEP_LOW_P, BYTE);
  Serial.print(BEEP_LOW_E, BYTE);
  Serial.print(BEEP_LOW_T, BYTE);
  delay(LCD_TONE_DELAY);
  delay(1000UL);
  
  Serial.print(LCD_LOCATE, BYTE);
  Serial.print(24, BYTE);
  Serial.print(24, BYTE);
  n = snprintf(strbuf, 7, " Set! ");
  Serial.print(LCD_STRING, BYTE);
  Serial.print(n, BYTE);
  for (byte i=0;  i<n;  ++i) {
    Serial.print(strbuf[i], BYTE);
  }
  Serial.print(LCD_TONE, BYTE);
  Serial.print(BEEP_LOW_P, BYTE);
  Serial.print(BEEP_LOW_E, BYTE);
  Serial.print(BEEP_LOW_T, BYTE);
  delay(LCD_TONE_DELAY);
  
  // Calibrate sensors
  for (unsigned long timeout = CALIBRATE_MS; timeout > 0; timeout -= CALIBRATE_MS / 10) {
    delay(CALIBRATE_MS / 10);
    pollFSRs();
    for(byte i=0;  i<FSR_N;  ++i) {
      fsrs_zero[i] = (fsrs_zero[i] == 0) ? fsrs[i] : (fsrs[i] + fsrs_zero[i])/2;
    }  
  }  
  
  Serial.print(LCD_LOCATE, BYTE);
  Serial.print(24, BYTE);
  Serial.print(24, BYTE);
  n = snprintf(strbuf, 7, "  Go! ");
  Serial.print(LCD_STRING, BYTE);
  Serial.print(n, BYTE);
  for (byte i=0;  i<n;  ++i) {
    Serial.print(strbuf[i], BYTE);
  }
  Serial.print(LCD_TONE, BYTE);
  Serial.print(BEEP_LOW_P, BYTE);
  Serial.print(BEEP_LOW_E, BYTE);
  Serial.print(BEEP_LOW_T, BYTE);
  delay(LCD_TONE_DELAY);  
  
  Serial.print(LCD_CLEAR, BYTE);
  delay(LCD_CLEAR_DELAY);

}

/***************************************************************************
 ***************************************************************************/

void outro() {
  Serial.print(LCD_CLEAR, BYTE);
  delay(LCD_CLEAR_DELAY);
  
  Serial.print(LCD_LOCATE, BYTE);
  Serial.print(16, BYTE);
  Serial.print(8, BYTE);

  char strbuf[16];
  int n = snprintf(strbuf, 9, " FINISH!");
  Serial.print(LCD_STRING, BYTE);
  Serial.print(n, BYTE);
  for (byte i=0;  i<n;  ++i) {
    Serial.print(strbuf[i], BYTE);
  }
  delay(1000UL); 
  
  Serial.print(LCD_LOCATE, BYTE);
  Serial.print(8, BYTE);
  Serial.print(24, BYTE);

  n = snprintf(strbuf, 11, "Score: %3.3d",model.score);
  Serial.print(LCD_STRING, BYTE);
  Serial.print(n, BYTE);
  for (byte i=0;  i<n;  ++i) {
    Serial.print(strbuf[i], BYTE);
  }
  Serial.print(LCD_TONE, BYTE);
  Serial.print(BEEP_LOW_P, BYTE);
  Serial.print(BEEP_LOW_E, BYTE);
  Serial.print(BEEP_LOW_T, BYTE);
  delay(LCD_TONE_DELAY);
  delay(2000UL);
}

/***************************************************************************
 ***************************************************************************/

void setup() {
  delay(1000);  // This must be here so we can program the device
  
  setupControl();

  // grab lower bits of sensor input to seed rand
  byte low_mask = B00001111;
  int seed = (int(lowByte(analogRead(FSR_PIN_LT)) && low_mask) << 12) |
    (int(lowByte(analogRead(FSR_PIN_RT)) && low_mask) << 8) |
    (int(lowByte(analogRead(FSR_PIN_LH)) && low_mask) << 4) |
    int(lowByte(analogRead(FSR_PIN_RH)) && low_mask);
  randomSeed(seed);  

  setupTimers();
}

/***************************************************************************
 ***************************************************************************/
volatile byte dummy;
void loop() {
  switch (mode) {
   case INTRO:
     setupView();
     setupModel();
     intro();
     startTimers();
     start_millis = millis();
     mode = RUN;
     break;
    case RUN:
      delay(LOOP_DELAY);
      pollFSRs();
      updateModel();
      updateView();
      break;
   case OUTRO:
     clearLEDs();
     delay(100);
     stopTimers();
     outro();
     mode = INTRO;
     break;
   }     
}

/***************************************************************************
 ***************************************************************************/
 
