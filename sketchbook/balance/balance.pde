
/***************************************************************************
 * Description:    FSR Balance Assessor
 * Authors:        Lisa Glendenning, Stephen Friedman
 * Last Modified:  March 2009
 ***************************************************************************/
 
#include <stdio.h>   // for snprintf
#include <string.h>  // for memset

// for timer interrupts
#include <avr/interrupt.h>
#include <avr/io.h>

#define DEBUG (0)

// use to index an array in two dimensions
#define ROWCOL(r,c,nc) ((r)*(nc)+(c))

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
#define FSR_LH (2)
#define FSR_RT (1)
#define FSR_RH (3)

// FSR input pins
#define FSR_PIN_LT (3)
#define FSR_PIN_LH (2)
#define FSR_PIN_RT (1)
#define FSR_PIN_RH (0)

// this is not currently used
#define FSR_SAMPLE_HZ (10)

// current value of FSRs
volatile int fsrs[] = {0, 0, 0, 0};

// LED pins
// To turn on an LED, set row pin to LOW and col pin to HIGH
#define LED_NPINS (6)

const int LED_ROW_PINS[] = {
  2, 3, 4, 5, 6, 7};
const int LED_COL_PINS[] = {
  8, 9, 10, 11, 12, 13};
  
// AVR timer loads
int timer1_load;
int timer2_load;

/***************************************************************************
 * View
 ***************************************************************************/

// LCD dimensions
#define LCD_XMIN (0)
#define LCD_YMIN (0)
#define LCD_XMAX (83)
#define LCD_YMAX (47)

// LCD character dimensions
#define LCD_X_PER_CHAR (6)
#define LCD_Y_PER_CHAR (8)

// LCD serial commands
#define LCD_BPS (9600)
#define LCD_LOCATE ('P')   // set pointer location
#define LCD_CLEAR  ('N')   // clears screen and moves pointer to (0,0)
#define LCD_LINE   ('L')   // draw line from pointer
#define LCD_TONE   ('T')   // audio tone
#define LCD_STRING ('S')   // ASCII text
#define LCD_BITMAP ('B')   // draw bitmap
#define LCD_SET_CONTRAST ('C')   // set the screen contrast

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

// LCD contrast value that seems to work
// well for our input voltage of 3.3V
// However, it is dependent on temp.
#define LCD_CONTRAST (0x00)

#define LED_MAP_NROWS (6)
#define LED_MAP_NCOLS (6)
#define LED_MAP_NPLANES (2)
#define LED_NUM_PLANE_CYCLES (7)
#define LED_MAP_PLANE_LOW  (0)
#define LED_MAP_PLANE_HIGH (1)

byte led_plane_sequence = B01111100;
volatile byte led_plane_cycle = 0;
volatile byte led_map[LED_MAP_NPLANES][LED_MAP_NCOLS];
volatile byte led_map_row = 0;

/***************************************************************************
 * Model
 ***************************************************************************/

/** From Bramberg et al. it looks like Parkinsons is ~4-10Hz wobbling
    That means our nyquist frequency is 20Hz, so any sampling rate above
    that should work.  This is a rough estimate from a single graph that
    should be verified through experimentation before use in a study */

#define MODEL_TIME_N (20)
#define MODEL_FRESH_N (10)
#define MODEL_FRESH_THRESHOLD (1)
#define MODEL_OFF_THRESHOLD (18)

#define MODEL_STATE_TWOFOOT   (0)
#define MODEL_STATE_LEFTFOOT  (1)
#define MODEL_STATE_RIGHTFOOT (2)

typedef struct balance_model_t {
  volatile boolean state_transit;
  volatile unsigned long tick;
  volatile int fsrs_tail;
  volatile int fsrs[MODEL_TIME_N*FSR_N];
  volatile long fsrs_accs[MODEL_TIME_N*FSR_N];
  volatile long fsrs_acc[FSR_N];
  volatile long wobble_acc[FSR_N];
  
  byte state;
  int fsrs_mu[FSR_N];
  int wobble_mu[FSR_N];
  int fsrs_fresh_mu[FSR_N];  
} balance_model_t;

balance_model_t model;

unsigned long walltime_start = 0;

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
  
  for (byte i=0;  i<LED_MAP_NPLANES;  ++i) {
    for (byte j=0;  j<LED_MAP_NCOLS;  ++j) {
      led_map[i][j] = 0;  
    }
  }
  
  Serial.begin(LCD_BPS);
}

/***************************************************************************
 ***************************************************************************/

static inline void pollFSRs() {
  // Read sensor value in the range [0,1023]
  // where a lower number is higher pressure
  // Flip values so increased reading corresponds to increased force
  // the relationship is still non-linear (inverse power law)
  fsrs[FSR_LT] = abs(analogRead(FSR_PIN_LT) - FSR_MAX);
  fsrs[FSR_LH] = abs(analogRead(FSR_PIN_LH) - FSR_MAX);
  fsrs[FSR_RT] = abs(analogRead(FSR_PIN_RT) - FSR_MAX);
  fsrs[FSR_RH] = abs(analogRead(FSR_PIN_RH) - FSR_MAX);
}

/***************************************************************************
 ***************************************************************************/

static inline void clearHistory() {
  model.tick = 0;
  // can't use memset on volatile memory
  for (byte i=0;  i<MODEL_TIME_N;  ++i) {
    for (byte j=0;  j<FSR_N;  ++j) {
      model.fsrs[ROWCOL(i,j,FSR_N)] = 0;
      model.fsrs_accs[ROWCOL(i,j,FSR_N)] = 0;
    }
  }
  for (byte i=0;  i<FSR_N;  ++i) {
    model.wobble_acc[i] = 0;
  }
  walltime_start = millis();
}  

/***************************************************************************
 ***************************************************************************/

/*
 * Update circular buffer and accumulators with new data
 */
static inline void timestep() {
  
  pollFSRs();
  
  if (model.state_transit) {
    clearHistory();
    model.state_transit = false;
  }
  
  // TODO this will overflow depending on the timestamp granularity
  model.tick++;
  
  // This will adjust the sample rate, % 4 for now
  if ((model.tick & B0011) == 0) {
    // add sample to head
    int model_fsrs_old_tail[FSR_N];
    for (byte i=0;  i<FSR_N;  ++i) {
      model_fsrs_old_tail[i] = model.fsrs[ROWCOL(model.fsrs_tail,i,FSR_N)]; 
      model.fsrs[ROWCOL(model.fsrs_tail,i,FSR_N)] = fsrs[i]; 
    }

    // update running accumulators
    for (byte i=0;  i<FSR_N;  ++i) {
      model.fsrs_acc[i] += fsrs[i] - model_fsrs_old_tail[i];
    }

    int model_accs_old_tail[FSR_N];
    for (byte i=0;  i<FSR_N;  ++i) {
      model_accs_old_tail[i] = model.fsrs_accs[ROWCOL(model.fsrs_tail,i,FSR_N)]; 
      model.fsrs_accs[ROWCOL(model.fsrs_tail,i,FSR_N)] = model.fsrs_acc[i]; 
    }
    for (byte i=0;  i<FSR_N;  ++i) {
      model.wobble_acc[i] += abs(fsrs[i] - model.fsrs_acc[i]/MODEL_TIME_N) - 
        abs(model_fsrs_old_tail[i]- model_accs_old_tail[i]/MODEL_TIME_N);
    }
  
    // increment tail pointer
    model.fsrs_tail++;
    if (model.fsrs_tail >= MODEL_TIME_N) {
      model.fsrs_tail = 0;
    }  
  }
} 

/***************************************************************************
 ***************************************************************************/

void updateModel() {
  // calculate running averages
  int n = (model.tick < MODEL_TIME_N) ? model.tick : MODEL_TIME_N;
  for (byte i=0;  i<FSR_N;  ++i) {
    model.fsrs_mu[i] = model.fsrs_acc[i] / n;
    model.wobble_mu[i] =  model.wobble_acc[i] / n;
  }
  
  // calculate fresh mu
  n = (model.tick < MODEL_FRESH_N) ? model.tick : MODEL_FRESH_N;
  for (byte i=0;  i<FSR_N;  ++i) {
    long model_fsrs_fresh_acc = 0;
    int index = model.fsrs_tail;
    for (byte t=1;  t<=n;  ++t) {
      index -= t;
      if (index < 0) {
        index = MODEL_TIME_N - 1;
      }
      model_fsrs_fresh_acc += model.fsrs[ROWCOL(index,i,FSR_N)];
    }
    model.fsrs_fresh_mu[i] = model_fsrs_fresh_acc / n;
  }
  
  // check if we are transitioning foot states
  if (model.state == MODEL_STATE_TWOFOOT) {  
    if ((model.fsrs_fresh_mu[FSR_LT] < MODEL_OFF_THRESHOLD) && 
        (model.fsrs_fresh_mu[FSR_LH] < MODEL_OFF_THRESHOLD)) {
      model.state_transit = true;
      model.state = MODEL_STATE_RIGHTFOOT;
    } else if ((model.fsrs_fresh_mu[FSR_RT] < MODEL_OFF_THRESHOLD) && 
               (model.fsrs_fresh_mu[FSR_RH] < MODEL_OFF_THRESHOLD)) {
      model.state_transit = true;
      model.state = MODEL_STATE_LEFTFOOT;
    }
  } else if (model.state == MODEL_STATE_RIGHTFOOT) {
    if ((model.fsrs_fresh_mu[FSR_LT] > MODEL_OFF_THRESHOLD) && 
        (model.fsrs_fresh_mu[FSR_LH] > MODEL_OFF_THRESHOLD)) {
      model.state_transit = true;
      model.state = MODEL_STATE_TWOFOOT;
    }    
  } else {
    if ((model.fsrs_fresh_mu[FSR_RT] > MODEL_OFF_THRESHOLD) && 
        (model.fsrs_fresh_mu[FSR_RH] > MODEL_OFF_THRESHOLD)) {
      model.state_transit = true;
      model.state = MODEL_STATE_TWOFOOT;
    }     
  }  
}

/***************************************************************************
 ***************************************************************************/

void setupModel() {
  model.state_transit = true;
  model.tick = 0;
  model.fsrs_tail = 0;
  model.state = MODEL_STATE_TWOFOOT;
}

/***************************************************************************
 ***************************************************************************/

void debugOutput() {
  unsigned long walltime = millis() - walltime_start;
  char freq_unit = ' ';
  unsigned long freq_scaled = 0; 
  if (model.tick < walltime) {
    freq_scaled = 1000*model.tick / walltime;    
  } else {
    freq_scaled = model.tick / walltime;  
    freq_unit = 'k';
    
    if (freq_scaled > 1000) {
      freq_scaled /= 1000;
      freq_unit = 'M';  
    }
  }
  
  Serial.print(LCD_LOCATE, BYTE);
  Serial.print(0*LCD_X_PER_CHAR, BYTE);
  Serial.print(2*LCD_Y_PER_CHAR, BYTE);
  
  char strbuf[16];
  int n = snprintf(strbuf, 16, "%3d", freq_scaled);
  n += snprintf(strbuf+3, 16, "%cHz ", freq_unit);
  Serial.print(LCD_STRING, BYTE);
  Serial.print(n, BYTE);
  for (byte i=0;  i<n;  ++i) {
    Serial.print(strbuf[i], BYTE);
  }  
  delay(LCD_STRING_DELAY);    

  if (model.state == MODEL_STATE_TWOFOOT) {
    strbuf[0] = '2';
  } else if (model.state == MODEL_STATE_RIGHTFOOT) {
    strbuf[0] = 'R';
  } else {
    strbuf[0] = 'L';
  }
  strbuf[1] = 'F';
  strbuf[2] = '\0';
  n = 2;

  Serial.print(LCD_STRING, BYTE);
  Serial.print(n, BYTE);
  for (byte i=0;  i<n;  ++i) {
    Serial.print(strbuf[i], BYTE);
  }  
  delay(LCD_STRING_DELAY);  
}
  
/***************************************************************************
 ***************************************************************************/
 
void updateView() {
  // ratio fresh mu and long mu to calculate LED levels
  // there's got to be a more elegant way to do this...
  for (byte i=0;  i<LED_MAP_NPLANES;  ++i) {
    for (byte j=0;  j<LED_MAP_NCOLS;  ++j) {
      led_map[i][j] = 0;  
    }
  }
  for (byte i=0;  i<FSR_N;  ++i) {
    byte col_center = 1 + (i%2)*(LED_MAP_NCOLS/2);
    byte row_center = 1 + ((i<2) ? (LED_MAP_NROWS/2) : 0); 
    int r = 100.0*(float(model.fsrs_fresh_mu[i]) / float(model.fsrs_mu[i]));
    if (r >= 100-MODEL_FRESH_THRESHOLD*5) {
      bitSet(led_map[LED_MAP_PLANE_LOW][col_center], row_center);
    }
    if (r >= 100-MODEL_FRESH_THRESHOLD*3) {
      bitSet(led_map[LED_MAP_PLANE_HIGH][col_center], row_center);
    }
    if (r >= 100-MODEL_FRESH_THRESHOLD) {
      bitSet(led_map[LED_MAP_PLANE_LOW][col_center-1], row_center);
      bitSet(led_map[LED_MAP_PLANE_LOW][col_center+1], row_center);
      bitSet(led_map[LED_MAP_PLANE_LOW][col_center], row_center+1);
      bitSet(led_map[LED_MAP_PLANE_LOW][col_center], row_center-1);
    }    
    if (r >= 100+MODEL_FRESH_THRESHOLD) {
      bitSet(led_map[LED_MAP_PLANE_HIGH][col_center-1], row_center);
      bitSet(led_map[LED_MAP_PLANE_HIGH][col_center+1], row_center);
      bitSet(led_map[LED_MAP_PLANE_HIGH][col_center], row_center-1);
      bitSet(led_map[LED_MAP_PLANE_HIGH][col_center], row_center+1);
    }     
    if (r >= 100+MODEL_FRESH_THRESHOLD*3) {
      bitSet(led_map[LED_MAP_PLANE_LOW][col_center-1], row_center-1);
      bitSet(led_map[LED_MAP_PLANE_LOW][col_center-1], row_center+1);
      bitSet(led_map[LED_MAP_PLANE_LOW][col_center+1], row_center-1);
      bitSet(led_map[LED_MAP_PLANE_LOW][col_center+1], row_center+1);
    }
    if (r >= 100+MODEL_FRESH_THRESHOLD*5) {
      bitSet(led_map[LED_MAP_PLANE_HIGH][col_center-1], row_center-1);
      bitSet(led_map[LED_MAP_PLANE_HIGH][col_center-1], row_center+1);
      bitSet(led_map[LED_MAP_PLANE_HIGH][col_center+1], row_center-1);
      bitSet(led_map[LED_MAP_PLANE_HIGH][col_center+1], row_center+1);
    }    
  }
  
  Serial.print(LCD_LOCATE, BYTE);
  Serial.print(0*LCD_X_PER_CHAR, BYTE);
  Serial.print(0*LCD_Y_PER_CHAR, BYTE);

  char strbuf[16];
  int n = snprintf(strbuf, 16, "%3d", model.fsrs_mu[FSR_LT]);
  Serial.print(LCD_STRING, BYTE);
  Serial.print(n, BYTE);
  for (byte i=0;  i<n;  ++i) {
    Serial.print(strbuf[i], BYTE);
  }  
  delay(LCD_STRING_DELAY);
  
  Serial.print(LCD_LOCATE, BYTE);
  Serial.print(0*LCD_X_PER_CHAR, BYTE);
  Serial.print(1*LCD_Y_PER_CHAR, BYTE);

  n = snprintf(strbuf, 16, "%3d", model.wobble_mu[FSR_LT]);
  Serial.print(LCD_STRING, BYTE);
  Serial.print(n, BYTE);
  for (byte i=0;  i<n;  ++i) {
    Serial.print(strbuf[i], BYTE);
  }  
  delay(LCD_STRING_DELAY);  
  
  Serial.print(LCD_LOCATE, BYTE);
  Serial.print(11*LCD_X_PER_CHAR, BYTE);
  Serial.print(0*LCD_Y_PER_CHAR, BYTE);

  n = snprintf(strbuf, 16, "%3d", model.fsrs_mu[FSR_RT]);
  Serial.print(LCD_STRING, BYTE);
  Serial.print(n, BYTE);
  for (byte i=0;  i<n;  ++i) {
    Serial.print(strbuf[i], BYTE);
  }  
  delay(LCD_STRING_DELAY);
  
  Serial.print(LCD_LOCATE, BYTE);
  Serial.print(11*LCD_X_PER_CHAR, BYTE);
  Serial.print(1*LCD_Y_PER_CHAR, BYTE);

  n = snprintf(strbuf, 16, "%3d\n", model.wobble_mu[FSR_RT]);
  Serial.print(LCD_STRING, BYTE);
  Serial.print(n, BYTE);
  for (byte i=0;  i<n;  ++i) {
    Serial.print(strbuf[i], BYTE);
  }  
  delay(LCD_STRING_DELAY);  
  
  Serial.print(LCD_LOCATE, BYTE);
  Serial.print(0*LCD_X_PER_CHAR, BYTE);
  Serial.print(4*LCD_Y_PER_CHAR, BYTE);

  n = snprintf(strbuf, 16, "%3d", model.fsrs_mu[FSR_LH]);
  Serial.print(LCD_STRING, BYTE);
  Serial.print(n, BYTE);
  for (byte i=0;  i<n;  ++i) {
    Serial.print(strbuf[i], BYTE);
  }  
  delay(LCD_STRING_DELAY);
  
  Serial.print(LCD_LOCATE, BYTE);
  Serial.print(0*LCD_X_PER_CHAR, BYTE);
  Serial.print(5*LCD_Y_PER_CHAR, BYTE);

  n = snprintf(strbuf, 16, "%3d", model.wobble_mu[FSR_LH]);
  Serial.print(LCD_STRING, BYTE);
  Serial.print(n, BYTE);
  for (byte i=0;  i<n;  ++i) {
    Serial.print(strbuf[i], BYTE);
  }  
  delay(LCD_STRING_DELAY);    
  
  Serial.print(LCD_LOCATE, BYTE);
  Serial.print(11*LCD_X_PER_CHAR, BYTE);
  Serial.print(4*LCD_Y_PER_CHAR, BYTE);

  n = snprintf(strbuf, 16, "%3d", model.fsrs_mu[FSR_RH]);
  Serial.print(LCD_STRING, BYTE);
  Serial.print(n, BYTE);
  for (byte i=0;  i<n;  ++i) {
    Serial.print(strbuf[i], BYTE);
  }  
  delay(LCD_STRING_DELAY);
  
  Serial.print(LCD_LOCATE, BYTE);
  Serial.print(11*LCD_X_PER_CHAR, BYTE);
  Serial.print(5*LCD_Y_PER_CHAR, BYTE);

  n = snprintf(strbuf, 16, "%3d", model.wobble_mu[FSR_RH]);
  Serial.print(LCD_STRING, BYTE);
  Serial.print(n, BYTE);
  for (byte i=0;  i<n;  ++i) {
    Serial.print(strbuf[i], BYTE);
  }  
  delay(LCD_STRING_DELAY);     
  
  if (DEBUG) {
    debugOutput();  
  }
}

/***************************************************************************
 ***************************************************************************/

void setupView() {
  Serial.print(LCD_CLEAR, BYTE);
  delay(LCD_CLEAR_DELAY);

  //Serial.print(LCD_SET_CONTRAST, BYTE);
  //Serial.print(LCD_CONTRAST);
  
  Serial.print(LCD_LOCATE, BYTE);
  Serial.print(4*LCD_X_PER_CHAR, BYTE);
  Serial.print(0*LCD_Y_PER_CHAR, BYTE);

  char strbuf[16];
  int n = snprintf(strbuf, 16, "FORCE");
  Serial.print(LCD_STRING, BYTE);
  Serial.print(n, BYTE);
  for (byte i=0;  i<n;  ++i) {
    Serial.print(strbuf[i], BYTE);
  }  
  delay(LCD_STRING_DELAY);
  
  Serial.print(LCD_LOCATE, BYTE);
  Serial.print(4*LCD_X_PER_CHAR, BYTE);
  Serial.print(4*LCD_Y_PER_CHAR, BYTE);

  Serial.print(LCD_STRING, BYTE);
  Serial.print(n, BYTE);
  for (byte i=0;  i<n;  ++i) {
    Serial.print(strbuf[i], BYTE);
  }  
  delay(LCD_STRING_DELAY);  
  
  Serial.print(LCD_LOCATE, BYTE);
  Serial.print(4*LCD_X_PER_CHAR, BYTE);
  Serial.print(1*LCD_Y_PER_CHAR, BYTE);
  
  n = snprintf(strbuf, 16, "WOBBLE");
  Serial.print(LCD_STRING, BYTE);
  Serial.print(n, BYTE);
  for (byte i=0;  i<n;  ++i) {
    Serial.print(strbuf[i], BYTE);
  }   
  delay(LCD_STRING_DELAY); 
  
  Serial.print(LCD_LOCATE, BYTE);
  Serial.print(4*LCD_X_PER_CHAR, BYTE);
  Serial.print(5*LCD_Y_PER_CHAR, BYTE);

  Serial.print(LCD_STRING, BYTE);
  Serial.print(n, BYTE);
  for (byte i=0;  i<n;  ++i) {
    Serial.print(strbuf[i], BYTE);
  }   
  delay(LCD_STRING_DELAY);   
}

/***************************************************************************
 ***************************************************************************/

// timer1 interrupt handler
ISR(TIMER1_OVF_vect, ISR_NOBLOCK) {
  timestep();

  //Capture the current timer value. This is how much error we
  //have due to interrupt latency and the work in this function
  byte latency = TCNT1;

  //Reload the timer and correct for latency.
  TCNT1 = latency + timer1_load;
}

/***************************************************************************
 ***************************************************************************/

// timer2 interrupt handler
ISR(TIMER2_OVF_vect) {

  // Update the row and plane for time-multiplexed grey scale
  byte old_row = led_map_row;
  led_map_row++;
  if (led_map_row >= LED_MAP_NROWS) {
    led_map_row = 0;
    led_plane_cycle++;
    if (led_plane_cycle >= LED_NUM_PLANE_CYCLES) { 
      led_plane_cycle = 0;
    }
  }
  byte plane = bitRead(led_plane_sequence, led_plane_cycle);

  // Merge new output with existing values
  byte new_output = (PORTB & B11000000) | 
    (led_map[plane][led_map_row] & B00111111);

  // Display the next column
  byte mask_high = 0x01 << LED_ROW_PINS[old_row];
  byte mask_low = ~(0x01 << LED_ROW_PINS[led_map_row]);

  PORTD |= mask_high;
  PORTB = new_output;
  PORTD &= mask_low;

  // Capture the current timer value. This is how much error we
  // have due to interrupt latency and the work in this function
  byte latency = TCNT2;

  // Reload the timer and correct for latency.
  TCNT2 = latency + timer2_load;
}

/***************************************************************************
 ***************************************************************************/

#define TIMER1_CLOCK_FREQ  (32150.0) //1MHz for /256 prescale from 8MHz
#define TIMER2_CLOCK_FREQ  (32150.0) //32kHz for /256 prescale from 8MHz

#define TIMER1_FREQ (10.0)
#define TIMER2_FREQ (20.0)

//Configures the ATMega168 8-Bit Timer1 and Timer2 to generate timed interrupts
//Sets the timer load value which must be loaded into TCNT1 and TCNT2
//inside the ISR routine.
// We will use timer1 for FSR sampling and timer2 for the LEDs
void setupTimers() {

  // Calculate the timer load values
  timer1_load = (int)((65536.0-(TIMER1_CLOCK_FREQ/TIMER1_FREQ))+0.5);
  timer2_load = (int)((256.0-(TIMER2_CLOCK_FREQ/TIMER2_FREQ))+0.5);

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
  TIMSK1 = 1<<TOIE1;  //Timer1 Overflow Interrupt Enable
  
  TCCR2A = 0;
  TCCR2B = 1<<CS22 | 0<<CS21 | 0<<CS20;
  TIMSK2 = 1<<TOIE2;  //Timer2 Overflow Interrupt Enable 

  // load the timers for the first cycle
  TCNT1 = timer1_load;
  TCNT2 = timer2_load;
  
  // ?
  //timer1_load = 0xFF80;
  timer1_load = 0xFD80;
}

/***************************************************************************
 ***************************************************************************/

void setup() {
  delay(1000);

  setupControl();
  setupView();
  setupModel();
  setupTimers();
}

/***************************************************************************
 ***************************************************************************/

void loop() {
  updateModel();
  updateView();
}

/***************************************************************************
 ***************************************************************************/
