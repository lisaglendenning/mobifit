
/***************************************************************************
 * Description:    FSR Etch-a-Sketch
 * Authors:        Lisa Glendenning, Stephen Friedman
 * Last Modified:  February 2009
 ***************************************************************************/

#include <stdio.h>

#define DEBUG (0)
#define LOOP_DELAY (10UL)  // ms

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
#define CALIBRATE_MS (5000UL)

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

#define BITMAP_NROWS (6)
#define BITMAP_NCOLS (84)
#define BITMAP_Y_PER_ROW (8)

byte bitmap[BITMAP_NROWS*BITMAP_NCOLS];

#define CURSOR_NROWS (1)
#define CURSOR_NCOLS (8)

byte cursor_bmp[CURSOR_NROWS*CURSOR_NCOLS] = {
   B00000000,B00011000, B00011000, B01111110, B01111110, B00011000, B00011000,B00000000};

// LED pins
// To turn on an LED, set row pin to LOW and col pin to HIGH
#define LED_NPINS (6)

const int LED_ROW_PINS[] = {
  2, 3, 4, 5, 6, 7};
const int LED_COL_PINS[] = {
  8, 9, 10, 11, 12, 13};

/***************************************************************************
 * Model
 ***************************************************************************/

#define MODEL_N  (-1)
#define MODEL_E  (1)
#define MODEL_S  (1)
#define MODEL_W  (-1)

#define MODEL_X  (0)
#define MODEL_Y  (1)

#define MODEL_TMAX (1)

#define MODEL_MAG_MIN (10)

#define MODEL_XSTART (42)
#define MODEL_YSTART (24)

int model_pos[MODEL_TMAX+1][2];

/***************************************************************************
 ***************************************************************************/

void setupControl() {
  pinMode(FSR_PIN_LT, INPUT);
  pinMode(FSR_PIN_LH, INPUT);
  pinMode(FSR_PIN_RT, INPUT);
  pinMode(FSR_PIN_RH, INPUT);
}

/***************************************************************************
 ***************************************************************************/

void pollControl() {
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
  for (byte i=0;  i<=MODEL_TMAX;  ++i) {
    model_pos[i][MODEL_X] = MODEL_XSTART;
    model_pos[i][MODEL_Y] = MODEL_YSTART;
  }
}

/***************************************************************************
 ***************************************************************************/

void debugModel(int fsr_sum_x, int fsr_sum_y) {
  Serial.print(LCD_CLEAR, BYTE);
  delay(LCD_CLEAR_DELAY);
/*
  // LED pulse
  digitalWrite(LED_ROW_PINS[0], LOW);
  digitalWrite(LED_COL_PINS[0], HIGH);
  digitalWrite(LED_COL_PINS[1], HIGH);
  digitalWrite(LED_COL_PINS[2], LOW);
  digitalWrite(LED_COL_PINS[3], HIGH);
  digitalWrite(LED_COL_PINS[4], HIGH);
  delay(500);
  digitalWrite(LED_COL_PINS[0], LOW);
*/
  char strbuf[16];
  int n = snprintf(strbuf, 15, "%d,%d\n", fsr_sum_x, fsr_sum_y);
  Serial.print(LCD_STRING, BYTE);
  Serial.print(n, BYTE);
  for (int i=0;  i<n;  ++i) {
    Serial.print(strbuf[i], BYTE);
  }

  delay(2000);
}

/***************************************************************************
 ***************************************************************************/

void updateModel() {
  byte i;

  // Shift positions back in time
  for (i=0;  i<MODEL_TMAX;  ++i) {
    model_pos[i][MODEL_X] = model_pos[i+1][MODEL_X];
    model_pos[i][MODEL_Y] = model_pos[i+1][MODEL_Y];
  }

  // Subtract "zero" values
  for(i=0;  i<FSR_N;  ++i) {
    fsrs[i] -= fsrs_zero[i];
  }

  // Each FSR reading corresponds to the magnitude A of a 2D vector
  // The directions, d, of the vectors are: LT=(-1,-1), LH=(-1,1),
  // RT=(1,-1), RH=(1,1)
  // Note: For a unit circle we would use sqrt(2)/2 instead of 1
  // We can represent each vector as a*d, where a = sqrt(A2 / 2)
  // Sum the vectors to determine the cartesian velocity

  int a[FSR_N];
  for (i=0;  i<FSR_N;  ++i) {
    a[i] = sqrt(sq(long(fsrs[i])) / 2L);
    if (fsrs[i] < 0) {
      a[i] = -a[i];
    }
  }

  int fsrs_sum[2];
  fsrs_sum[MODEL_X] = MODEL_W*(a[FSR_LT] + a[FSR_LH]) +
    MODEL_E*(a[FSR_RT] + a[FSR_RH]);
  fsrs_sum[MODEL_Y] = MODEL_N*(a[FSR_LT] + a[FSR_RT]) +
    MODEL_S*(a[FSR_LH] + a[FSR_RH]);

  // Threshold vector magnitude of fsrs_sum to ignore small vectors
  // where ||fsr_sum|| = sqrt(x2 + y+2)
  int fsrs_sum_mag = sqrt(sq(long(fsrs_sum[MODEL_X]))
    + sq(long(fsrs_sum[MODEL_Y])));
  if (fsrs_sum_mag > MODEL_MAG_MIN) {
    // We now ignore the magnitude and find coarse direction
    int dx = 0;
    int dy = 0;
    float xy_ratio = 0;

    // Find the closest unit vector
    if (fsrs_sum[MODEL_Y] == 0) {  // avoid divide by zero
      xy_ratio = 4;
    }
    else {
      xy_ratio = abs(float(fsrs_sum[MODEL_X]) / float(fsrs_sum[MODEL_Y]));
    }
    if(xy_ratio < 2) {
      if (fsrs_sum[MODEL_Y] >= 0) {
        dy = 1;
      }
      else {
        dy = -1;
      }
    }

    if (xy_ratio > 0.5) {
      if (fsrs_sum[MODEL_X] >= 0) {
        dx = 1;
      }
      else {
        dx = -1;
      }
    }

    // Update position
    model_pos[MODEL_TMAX][MODEL_X] = constrain(model_pos[MODEL_TMAX][MODEL_X]+dx,
    LCD_XMIN, LCD_XMAX);
    model_pos[MODEL_TMAX][MODEL_Y] = constrain(model_pos[MODEL_TMAX][MODEL_Y]+dy,
    LCD_YMIN, LCD_YMAX);
  }

  if (DEBUG) {
    debugModel(fsrs_sum[MODEL_X], fsrs_sum[MODEL_Y]);
  }
}

/***************************************************************************
 ***************************************************************************/

void setupView() {
  for (byte i=0;  i<LED_NPINS;  ++i) {
    pinMode(LED_ROW_PINS[i], OUTPUT);
    pinMode(LED_COL_PINS[i], OUTPUT);
  }

  byte row;
  byte col;
  for (row=0;  row<BITMAP_NROWS;  ++row) {
    for (col=0;  col<BITMAP_NCOLS;  ++col) {
      bitmap[ROWCOL(row,col,BITMAP_NCOLS)] = B00000000;
    }
  }
  col = MODEL_XSTART;
  row = MODEL_YSTART / BITMAP_Y_PER_ROW;
  byte row_offset = MODEL_YSTART % BITMAP_Y_PER_ROW;
  bitmap[ROWCOL(row,col,BITMAP_NCOLS)] |= B00000001 << row_offset;

  Serial.begin(LCD_BPS);

  Serial.print(LCD_CLEAR, BYTE);
  delay(LCD_CLEAR_DELAY);

  char strbuf[16];
  int n = snprintf(strbuf, 15, "Etch-a-Sketch!\n");
  Serial.print(LCD_STRING, BYTE);
  Serial.print(n, BYTE);
  for (byte i=0;  i<n;  ++i) {
    Serial.print(strbuf[i], BYTE);
  }
  delay(3000);

  Serial.print(LCD_CLEAR, BYTE);
  delay(LCD_CLEAR_DELAY);
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

void bitmap_layer(byte *bg, byte bg_nrows, byte bg_ncols, byte *fg,
    byte fg_nrows, byte fg_ncols, int xoffset, int yoffset, boolean add) {
  int xstart = xoffset - fg_ncols/2;
  int xend = xstart + fg_ncols;
  int ystart = yoffset - (fg_nrows*BITMAP_Y_PER_ROW)/2;
  int yend = ystart + fg_nrows*BITMAP_Y_PER_ROW;
  byte bg_col_start;
  byte bg_col_end;
  byte fg_col_offset;
  byte bg_row_start;
  byte bg_row_end;
  byte fg_row_offset;
  byte bg_y_shift;

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
    bg_y_shift = BITMAP_Y_PER_ROW - (abs(ystart) % BITMAP_Y_PER_ROW);
    fg_row_offset = ceil(float(abs(ystart)) / float(BITMAP_Y_PER_ROW));
    bg_row_start = 0;
  } else {
    bg_y_shift = ystart % BITMAP_Y_PER_ROW;
    fg_row_offset = 0;
    bg_row_start = ystart / BITMAP_Y_PER_ROW;
  }
  if (yend > bg_nrows*BITMAP_Y_PER_ROW) {
    bg_row_end = bg_nrows;
  } else {
    bg_row_end = ceil(float(yend) / float(BITMAP_Y_PER_ROW));
  }

  for (byte row=0;  bg_row_start+row<bg_row_end;  ++row) {
    for (byte col=0;  bg_col_start+col<bg_col_end;  ++col) {
      byte b = bg[ROWCOL(bg_row_start+row,bg_col_start+col,bg_ncols)];
      if ((row > 0) || (ystart < 0)) {
          byte fg_prev = fg[ROWCOL(fg_row_offset+row-1,fg_col_offset+col,fg_ncols)]
              >> (BITMAP_Y_PER_ROW - bg_y_shift);
          b = byte_layer(b, fg_prev, add);
      }

      if ((bg_row_start+row+1<bg_row_end) || (yend > bg_nrows*BITMAP_Y_PER_ROW) || (bg_y_shift == 0)) {
          byte fg_next = fg[ROWCOL(fg_row_offset+row,fg_col_offset+col,fg_ncols)] << bg_y_shift;
          b = byte_layer(b, fg_next, add);
      }

      Serial.print(LCD_LOCATE, BYTE);
      Serial.print(bg_col_start+col, BYTE);
      Serial.print((bg_row_start+row)*BITMAP_Y_PER_ROW, BYTE);
      delay(LCD_LOCATE_DELAY);
      Serial.print(LCD_BITMAP, BYTE);
      Serial.print(1, BYTE);
      Serial.print(b, BYTE);
      delay(LCD_BITMAP_DELAY);
    }
  }
}

/***************************************************************************
 ***************************************************************************/

void updateView() {
  // check if we are in new territory
  byte col = model_pos[MODEL_TMAX][MODEL_X];
  byte row = model_pos[MODEL_TMAX][MODEL_Y] / BITMAP_Y_PER_ROW;
  byte row_offset = model_pos[MODEL_TMAX][MODEL_Y] % BITMAP_Y_PER_ROW;
  byte bitmask = B00000001 << row_offset;
  if (!(bitmap[ROWCOL(row,col,BITMAP_NCOLS)] & bitmask)) {

    // we only move one pixel per tick
    bitmap[ROWCOL(row,col,BITMAP_NCOLS)] |= bitmask;

    // Redraw the changed byte
    Serial.print(LCD_LOCATE, BYTE);
    Serial.print(col, BYTE);
    Serial.print(row*BITMAP_Y_PER_ROW, BYTE);
    delay(LCD_LOCATE_DELAY);
    Serial.print(LCD_BITMAP, BYTE);
    Serial.print(1, BYTE);
    Serial.print(bitmap[ROWCOL(row,col,BITMAP_NCOLS)], BYTE);
    delay(LCD_BITMAP_DELAY);

    // And column zero, since it acts weird
    for (row=0;  row<BITMAP_NROWS;  ++row) {
      Serial.print(LCD_LOCATE, BYTE);
      Serial.print(0, BYTE);
      Serial.print(row*BITMAP_Y_PER_ROW, BYTE);
      delay(LCD_LOCATE_DELAY);
      Serial.print(LCD_BITMAP, BYTE);
      Serial.print(1, BYTE);
      Serial.print(bitmap[ROWCOL(row,0,BITMAP_NCOLS)], BYTE);
      delay(LCD_BITMAP_DELAY);
    }
  }


  // Unblink cursor
  bitmap_layer(bitmap, BITMAP_NROWS, BITMAP_NCOLS, cursor_bmp,
    CURSOR_NROWS, CURSOR_NCOLS, model_pos[MODEL_TMAX-1][MODEL_X],
    model_pos[MODEL_TMAX-1][MODEL_Y], true);
}

/***************************************************************************
 ***************************************************************************/

void calibrate() {
  char strbuf[16];
  int n = snprintf(strbuf, 15, "Calibrating...\n");
  Serial.print(LCD_STRING, BYTE);
  Serial.print(n, BYTE);
  for (byte i=0;  i<n;  ++i) {
    Serial.print(strbuf[i], BYTE);
  }
  delay(LCD_STRING_DELAY);

  unsigned long time = millis();
  unsigned long timeout = time + CALIBRATE_MS;
  while (time < timeout) {
    delay(100UL);
    pollControl();
    time = millis();
  }
  for(byte i=0;  i<FSR_N;  ++i) {
    fsrs_zero[i] = fsrs[i];
  }

  Serial.print(LCD_CLEAR, BYTE);
  delay(LCD_CLEAR_DELAY);

  Serial.print(LCD_LOCATE, BYTE);
  Serial.print(MODEL_XSTART, BYTE);
  Serial.print(MODEL_YSTART, BYTE);
  delay(LCD_LOCATE_DELAY);

  Serial.print(LCD_TONE, BYTE);
  Serial.print(BEEP_LOW_P, BYTE);
  Serial.print(BEEP_LOW_E, BYTE);
  Serial.print(BEEP_LOW_T, BYTE);
  delay(LCD_TONE_DELAY);
}

/***************************************************************************
 ***************************************************************************/

void setup() {
  delay(1000);

  setupControl();
  setupModel();
  setupView();

  calibrate();
}

/***************************************************************************
 ***************************************************************************/

void loop() {
  delay(LOOP_DELAY);
  pollControl();
  updateModel();
  if (!DEBUG) {
    updateView();
  }
}

/***************************************************************************
 ***************************************************************************/
