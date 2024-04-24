// This code is made using the very good sine wave freq detection by Amanda Ghassaei july 2012
// https://www.instructables.com/member/amandaghassaei/

// Changelog
// Code origin Sam / LookMumNoComputer and Amandaghassaei
// 9 Jan 2020: Jos Bouten aka Zaphod B: 
// - put frequencies in a table and simplified controlling the led display
// - put strings in flash memory to use less program memory space.

// 18 Jan 2020
// Added test of clipping led.

// 29 Febr 2020
// Added a switch mechanism to the code to make it easy to use either a common anode or common
// cathode LED-display.
// Set the const LED_DISPLAY_TYPE to the type of LED display you use (COMMON_ANODE or COMMON_CATHODE).

// Feb. Mar. 2024
// - new frequencies (calculated from script, tuned for A4=440Hz, with 6 cents tolerance)
// - modification of display chars (reduced memory usage and simplification)
// - added exponential weighted moving average calculation for the period, and increased precision

/*
 * This code reads an analog signal, finds the rising edge, measures the period between each rising edge,
 * computes the frequency, compares that frequency to the table of in-tune frequencies and finally displays
 * the corresponding note and if it's in tune or not.
 * 
 * Compatible hardware:
 * - https://www.lookmumnocomputer.com/projects#/1222-performance-vco
 * - https://github.com/MyModularJourney/Tuna
 * - https://www.davidhaillant.com/category/electronic-projects/utility-modules/tuner/
 * 
 * 
 * input DC biased (+2.5V) signal on pin A0
 * 
 * output one 7-segment display on pins D2 to D9 (see below)
 * output 3 LEDs on pins A3, A4 and A5 for
 */

/*
   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 3 of the License, or
   (at your option) any later version.
*/


// uncomment to activate serial output and LEDs test patterns
#define DEBUG

// uncomment the corresponding 7-segment display type
#define COMMON_ANODE
//#define COMMON_CATHODE


// choose your charset. Uncomment the following lines to select either large or small characters:
#define LARGE_G
//#define LARGE_B

// LARGE_G is shaped like "G", else it's shaped like "g"
// LARGE_B is shaped like "B", else it's shaped like "b"


// **** pin definitions (hardware dependent) ****

// "target" tuning LED output pins. Active HIGH (as Common Cathode)
#define LED_FLAT_PIN      18    // A4 (flat)
#define LED_IN_TUNE_PIN   19    // A5 (in-tune)
#define LED_SHARP_PIN     17    // A3 (sharp)

// 9 segment display output pins;
#define LEDA  8
#define LEDB  9
#define LEDC  4
#define LEDD  3
#define LEDE  2
#define LEDF  7
#define LEDG  6
#define LEDDP 5


// frequency detection variables
byte newData  = 0;
byte prevData = 0;

uint16_t period = 0;
uint32_t averaged_period = 0;

//uint16_t frequency = 0;

#define HALF_SAMPLE_VALUE 127

/* 
 * TIMER_RATE is ADC clock (set by prescaler) divided by number of cycles for each conversion: 
 * 500kHz / 13 = 38461,538461538
 * 
 * As we only want integers, but with better precision, TIMER_RATE is int((500kHz / 13) x 10)
 * To correct the 10x factor, TIMER_RATE_10 is equal to TIMER_RATE
 */

#define TIMER_RATE 384615
#define TIMER_RATE_10 TIMER_RATE

// Data storage variables.
unsigned int time = 0;    // Keeps time and sends values to store in timer[] occasionally.
#define BUFFER_SIZE 10
int timer[BUFFER_SIZE];   // Storage for timing of events.
int slope[BUFFER_SIZE];   // Storage for slope of events.
unsigned int totalTimer;  // Used to calculate period.
byte index = 0;           // Current storage index.
int maxSlope = 0;         // Used to calculate max slope as trigger point.
int newSlope;             // Storage for incoming slope data.

// Variables for decided whether you have a match.
#define MAX_NO_MATCH_VALUE 9
byte noMatch = 0;         // Counts how many non-matches you've received to reset variables if it's been too long.
byte slopeTol = 100;      // Slope tolerance - adjust this if you need.    *** 100 required here for dealing with square-ish signals ***
int timerTol = 10;        // Timer tolerance - adjust this if you need.

// Variables for amp detection.
unsigned int ampTimer = 0;
byte maxAmp = 0;
byte checkMaxAmp;
byte ampThreshold = 30;   // Raise if you have a very noisy signal.
long clippingTimer = 0;

// Clipping indicator variables.
boolean clipping = true;
#define CLIPPING_TIME 5 * TIMER_RATE // This should amount to 2 seconds.

#define write_pin12_low  PORTB &= B11101111
#define write_pin12_high PORTB |= B00010000


ISR(ADC_vect) {       // When new ADC value ready.
  write_pin12_low; // Set pin 12 low.
  prevData = newData; // Store previous value.
  newData = ADCH;     // Get value from A0.
  if (prevData < HALF_SAMPLE_VALUE && newData >= HALF_SAMPLE_VALUE){ // if increasing and crossing midpoint
    newSlope = newData - prevData; // Calculate slope
    if (abs(newSlope - maxSlope) < slopeTol){ // If slopes are ==
      // Record new data and reset time.
      slope[index] = newSlope;
      timer[index] = time;
      time = 0;
      if (index == 0){ // New max slope just reset.
        write_pin12_high; // Set pin 12 high.
        noMatch = 0;
        index++; // Increment index.
      }
      else if (abs(timer[0] - timer[index]) < timerTol && abs(slope[0] - newSlope) < slopeTol){ //if timer duration and slopes match
        // Sum timer values.
        totalTimer = 0;
        for (byte i = 0; i < index; i++){
          totalTimer += timer[i];
        }

        period = totalTimer * 100 ; // Set period. and increase resolution

        // moving average
        averaged_period = period + averaged_period - ((averaged_period - 8) >> 4);
        // remember to divide by 16 (>> 4)
        
        // Reset new zero index values to compare with.
        timer[0] = timer[index];
        slope[0] = slope[index];
        index = 1; // Set index to 1.
        write_pin12_high; // Set pin 12 high.
        noMatch = 0;
      } else { // Crossing midpoint but not match.
        index++; // Increment index.
        if (index > BUFFER_SIZE - 1){
          reset();
        }
      }
    }
    else if (newSlope > maxSlope){ // If new slope is much larger than max slope.
      maxSlope = newSlope;
      time = 0; // Reset clock.
      noMatch = 0;
      index = 0; // Reset index.
    }
    else { // Slope not steep enough.
      noMatch++; // Increment no match counter.
      if (noMatch > MAX_NO_MATCH_VALUE){
        reset();
      }
    }
  }
    
  if (newData == 0 || newData == 1023){ // If clipping 
    PORTB |= B00100000; // set pin 13 high, i.e. turn on clipping indicator led.
    clipping = true; // Currently clipping.
  }
  
  time++; // Increment timer at rate of 38.5kHz
  clippingTimer++;
  if (clippingTimer > CLIPPING_TIME) {
    PORTB &= B11011111; // Set pin 13 low, i.e. turn off clipping indicator led.
    clipping = false;   // Currently not clipping.
    clippingTimer = 0;
  }
  
  ampTimer++; // Increment amplitude timer.
  if (abs(HALF_SAMPLE_VALUE - ADCH) > maxAmp){
    maxAmp = abs(HALF_SAMPLE_VALUE-ADCH);
  }
  if (ampTimer == 1000){
    ampTimer = 0;
    checkMaxAmp = maxAmp;
    maxAmp = 0;
  }
}

void reset(){   // Clear out some variables.
  index = 0;    // Reset index.
  noMatch = 0;  // Reset match counter.
  maxSlope = 0; // Reset slope.
}




/*
 * Frequency data
 * Example of data generated (7 octaves from C0 (MIDI #12) to B6 (MIDI #95))
 * with 6 cents in-tune range:

  Note #: 12 Freq.: 163.52   | 159 | 163 | 164 | 168 |
  Note #: 13 Freq.: 173.24    | 168 | 173 | 174 | 178 |
  Note #: 14 Freq.: 183.54    | 178 | 183 | 184 | 189 |
  Note #: 15 Freq.: 194.45    | 189 | 194 | 195 | 200 |
  Note #: 16 Freq.: 206.02    | 200 | 205 | 207 | 212 |
  Note #: 17 Freq.: 218.27    | 212 | 218 | 219 | 225 |
  Note #: 18 Freq.: 231.25    | 225 | 230 | 232 | 238 |
  Note #: 19 Freq.: 245.00    | 238 | 244 | 246 | 252 |
  Note #: 20 Freq.: 259.57    | 252 | 259 | 260 | 267 |
  Note #: 21 Freq.: 275.00    | 267 | 274 | 276 | 283 |
  Note #: 22 Freq.: 291.35    | 283 | 290 | 292 | 300 |
  Note #: 23 Freq.: 308.68    | 300 | 308 | 310 | 318 |
  Note #: 24 Freq.: 327.03    | 318 | 326 | 328 | 337 |
  Note #: 25 Freq.: 346.48    | 337 | 345 | 348 | 357 |
  Note #: 26 Freq.: 367.08    | 357 | 366 | 368 | 378 |
  Note #: 27 Freq.: 388.91    | 378 | 388 | 390 | 400 |
  Note #: 28 Freq.: 412.03    | 400 | 411 | 413 | 424 |
  Note #: 29 Freq.: 436.54    | 424 | 435 | 438 | 449 |
  Note #: 30 Freq.: 462.49    | 449 | 461 | 464 | 476 |
  Note #: 31 Freq.: 489.99    | 476 | 488 | 492 | 504 |
  Note #: 32 Freq.: 519.13    | 504 | 517 | 521 | 534 |
  Note #: 33 Freq.: 550.00    | 534 | 548 | 552 | 566 |
  Note #: 34 Freq.: 582.70    | 566 | 581 | 585 | 600 |
  Note #: 35 Freq.: 617.35    | 600 | 615 | 619 | 635 |
  Note #: 36 Freq.: 654.06    | 635 | 652 | 656 | 673 |
  Note #: 37 Freq.: 692.96    | 673 | 691 | 695 | 713 |
  Note #: 38 Freq.: 734.16    | 713 | 732 | 737 | 756 |
  Note #: 39 Freq.: 777.82    | 756 | 775 | 781 | 801 |
  Note #: 40 Freq.: 824.07    | 801 | 821 | 827 | 848 |
  Note #: 41 Freq.: 873.07    | 848 | 870 | 876 | 899 |
  Note #: 42 Freq.: 924.99    | 899 | 922 | 928 | 952 |
  Note #: 43 Freq.: 979.99    | 952 | 977 | 983 | 1009  |
  Note #: 44 Freq.: 1038.26   | 1009  | 1035  | 1042  | 1069  |
  Note #: 45 Freq.: 1100.00   | 1069  | 1096  | 1104  | 1132  |
  Note #: 46 Freq.: 1165.41   | 1132  | 1161  | 1169  | 1200  |
  Note #: 47 Freq.: 1234.71   | 1200  | 1230  | 1239  | 1271  |
  Note #: 48 Freq.: 1308.13   | 1271  | 1304  | 1313  | 1346  |
  Note #: 49 Freq.: 1385.91   | 1346  | 1381  | 1391  | 1427  |
  Note #: 50 Freq.: 1468.32   | 1427  | 1463  | 1473  | 1511  |
  Note #: 51 Freq.: 1555.64   | 1511  | 1550  | 1561  | 1601  |
  Note #: 52 Freq.: 1648.14   | 1601  | 1642  | 1654  | 1696  |
  Note #: 53 Freq.: 1746.14   | 1696  | 1740  | 1752  | 1797  |
  Note #: 54 Freq.: 1849.97   | 1797  | 1844  | 1856  | 1904  |
  Note #: 55 Freq.: 1959.98   | 1904  | 1953  | 1967  | 2017  |
  Note #: 56 Freq.: 2076.52   | 2017  | 2069  | 2084  | 2137  |
  Note #: 57 Freq.: 2200.00   | 2137  | 2192  | 2208  | 2264  |
  Note #: 58 Freq.: 2330.82   | 2264  | 2323  | 2339  | 2399  |
  Note #: 59 Freq.: 2469.42   | 2399  | 2461  | 2478  | 2542  |
  Note #: 60 Freq.: 2616.26   | 2542  | 2607  | 2625  | 2693  |
  Note #: 61 Freq.: 2771.83   | 2693  | 2762  | 2781  | 2853  |
  Note #: 62 Freq.: 2936.65   | 2853  | 2926  | 2947  | 3023  |
  Note #: 63 Freq.: 3111.27   | 3023  | 3101  | 3122  | 3202  |
  Note #: 64 Freq.: 3296.28   | 3202  | 3285  | 3308  | 3393  |
  Note #: 65 Freq.: 3492.28   | 3393  | 3480  | 3504  | 3595  |
  Note #: 66 Freq.: 3699.94   | 3595  | 3687  | 3713  | 3808  |
  Note #: 67 Freq.: 3919.95   | 3808  | 3906  | 3934  | 4035  |
  Note #: 68 Freq.: 4153.05   | 4035  | 4139  | 4167  | 4275  |
  Note #: 69 Freq.: 4400.00   | 4275  | 4385  | 4415  | 4529  |
  Note #: 70 Freq.: 4661.64   | 4529  | 4646  | 4678  | 4798  |
  Note #: 71 Freq.: 4938.83   | 4798  | 4922  | 4956  | 5084  |
  Note #: 72 Freq.: 5232.51   | 5084  | 5214  | 5251  | 5386  |
  Note #: 73 Freq.: 5543.65   | 5386  | 5524  | 5563  | 5706  |
  Note #: 74 Freq.: 5873.30   | 5706  | 5853  | 5894  | 6045  |
  Note #: 75 Freq.: 6222.54   | 6045  | 6201  | 6244  | 6405  |
  Note #: 76 Freq.: 6592.55   | 6405  | 6570  | 6615  | 6786  |
  Note #: 77 Freq.: 6984.56   | 6786  | 6960  | 7009  | 7189  |
  Note #: 78 Freq.: 7399.89   | 7189  | 7374  | 7426  | 7617  |
  Note #: 79 Freq.: 7839.91   | 7617  | 7813  | 7867  | 8070  |
  Note #: 80 Freq.: 8306.09   | 8070  | 8277  | 8335  | 8549  |
  Note #: 81 Freq.: 8800.00   | 8549  | 8770  | 8831  | 9058  |
  Note #: 82 Freq.: 9323.27   | 9058  | 9291  | 9356  | 9596  |
  Note #: 83 Freq.: 9877.67   | 9596  | 9843  | 9912  | 10167 |
  Note #: 84 Freq.: 10465.02    | 10167 | 10429 | 10501 | 10772 |
  Note #: 85 Freq.: 11087.30    | 10772 | 11049 | 11126 | 11412 |
  Note #: 86 Freq.: 11746.59    | 11412 | 11706 | 11787 | 12091 |
  Note #: 87 Freq.: 12445.08    | 12091 | 12402 | 12488 | 12810 |
  Note #: 88 Freq.: 13185.10    | 12810 | 13139 | 13231 | 13571 |
  Note #: 89 Freq.: 13969.13    | 13571 | 13921 | 14018 | 14378 |
  Note #: 90 Freq.: 14799.78    | 14378 | 14749 | 14851 | 15233 |
  Note #: 91 Freq.: 15679.82    | 15233 | 15626 | 15734 | 16139 |
  Note #: 92 Freq.: 16612.19    | 16139 | 16555 | 16670 | 17099 |
  Note #: 93 Freq.: 17600.00    | 17099 | 17539 | 17661 | 18116 |
  Note #: 94 Freq.: 18646.55    | 18116 | 18582 | 18711 | 19193 |
  Note #: 95 Freq.: 19755.33    | 19193 | 19687 | 19824 | 20334 |

 * Generated data can be checked on serial monitor: enable DEBUG flag to visualize
 *
 */

// Define the Note struct
struct Note {
    uint16_t highestFrequency;  // Highest frequency for the note
    uint16_t lowestFrequency;   // Lowest frequency for the note
    uint16_t lowerRange;        // Lower range in tune
    uint16_t upperRange;        // Upper range in tune
};

// Define the number of octaves and notes per octave
const int numOctaves = 7;
const int numNotesPerOctave = 12;
const float cents = 6.0;

// Create a 2D array to hold notes for each octave and each note in the octave
Note notes[numOctaves][numNotesPerOctave];

// 4 x 2 bytes per Note struct
// x 12 per octave = 96 bytes
// x 7 for 7 octaves = 672 bytes

const float A4_reference = 4400.0;
// Function to calculate the frequency, in dHz (1/10 of Hz), of a note
// noteNumber is the MIDI note number, where C0 = 12 and A4 = 69
float calculateFrequency(int noteNumber)
{
  return A4_reference * pow(2.0, (noteNumber - 69) / 12.0);
}

// Function to calculate the frequency ranges for a given note's central frequency
void calculateRanges(float central_frequency, uint16_t &min_boundary, uint16_t &min_acceptable, uint16_t &max_acceptable, uint16_t &max_boundary)
{
  min_boundary   = round(central_frequency * pow(2.0, -50.0 / 1200.0));
  min_acceptable = round(central_frequency * pow(2.0, -cents / 1200.0));
  max_acceptable = round(central_frequency * pow(2.0, cents / 1200.0));
  max_boundary   = round(central_frequency * pow(2.0, 50.0 / 1200.0));
}

void setup_all_frequencies(void)
{
  float central_frequency = 0.0;
  
  uint16_t min_boundary = 0;
  uint16_t min_acceptable = 0;
  uint16_t max_acceptable = 0;
  uint16_t max_boundary = 0;

  uint8_t midiNoteNumber = 0;
  
  // Populate the notes array
  for (uint8_t octave = 0; octave < numOctaves; octave++)       // 0 to 6
  {
    for (uint8_t note = 0; note < numNotesPerOctave; note++)    // 0 to 11
    {
      // Calculate the MIDI note number
      midiNoteNumber = (octave * numNotesPerOctave) + note + 12;    // start at C0 = 12

      // Calculate the frequency for the note
      central_frequency = calculateFrequency(midiNoteNumber);

      // Calculate the frequency ranges
      calculateRanges(central_frequency, min_boundary, min_acceptable, max_acceptable, max_boundary);

      // Populate the Note struct
      notes[octave][note].highestFrequency = max_boundary;
      notes[octave][note].lowestFrequency = min_boundary;
      notes[octave][note].lowerRange = min_acceptable;
      notes[octave][note].upperRange = max_acceptable;

      #ifdef DEBUG
        Serial.print(F("Octave #: "));
        Serial.print(octave);
        Serial.print(F(" Note #: "));
        Serial.print(midiNoteNumber);
        Serial.print(F(" Freq.: "));
        Serial.print(central_frequency);
        Serial.print(F("  \t| "));
        Serial.print(min_boundary);
        Serial.print(F("\t| "));
        Serial.print(min_acceptable);
        Serial.print(F("\t| "));
        Serial.print(max_acceptable);
        Serial.print(F("\t| "));
        Serial.print(max_boundary);
        Serial.println(F("\t|"));
      #endif
    }
  }
}


// **** note detection *****************************
uint8_t find_octave(uint16_t frequency)
{
  uint8_t octave_found = 0xFF;
//  Serial.print(F(" O:"));

  for (uint8_t octave = 0; octave < numOctaves; octave++)       // 0 to 6
  {
//    Serial.print(octave);
//    Serial.print(F(" low:"));
//    Serial.print(notes[octave][0].lowestFrequency);
//    Serial.print(F(" high:"));
//    Serial.print(notes[octave][numNotesPerOctave - 1].highestFrequency);
//    Serial.print(F(" "));

    if ((frequency >= notes[octave][0].lowestFrequency) && (frequency < notes[octave][numNotesPerOctave - 1].highestFrequency))
    {
      octave_found = octave;
    }
  }
//  Serial.print(F(" found: "));
//  Serial.print(octave_found);
//  Serial.print(F("| "));

  return octave_found;
}

#define TUNING_FLAT 'b'
#define TUNING_IN_TUNE '-'
#define TUNING_SHARP '#'

uint8_t find_note(uint16_t frequency, uint8_t &octave, uint8_t &tuning)
// input:
// * frequency is in dHz (1/10 of Hertz)
// outputs:
// * octave will be the octave found (number from 0 to numOctaves), 0xFF if out-of-range
// * tuning will be the state of the current note (sharp, in-tune, or flat), or 0xFF if out-of-range
// return the note number if found, or 0xFF if out-of-range
{
  uint8_t note_found = 0xFF;

  // search in 2 stages faster than full array search (7 + 12 iterations)
  // first, find the octave
  octave = find_octave(frequency);

  if (octave != 0xFF)
  {
    // then, if octave has been identified, find the note within the octave
    for (uint8_t note = 0; note < numNotesPerOctave; note++)    // 0 to 11
    {
      if ((frequency >= notes[octave][note].lowestFrequency) && (frequency < notes[octave][note].highestFrequency))
      {
        note_found = note;
      }
    }

    // and then, find if the frequency is in tune, flat or sharp
    if (frequency < notes[octave][note_found].lowerRange)
    {
      // flat!
      tuning = TUNING_FLAT;
    }
    else
    {
      if (frequency > notes[octave][note_found].upperRange)
      {
        // sharp!
        tuning = TUNING_SHARP;
      }
      else
      {
        // in tune!
        tuning = TUNING_IN_TUNE;
      }
    }
  }

  // full array search, longer (84 iterations with 7 octaves...)
/*  for (uint8_t octave = 0; octave < numOctaves; octave++)       // 0 to 6
  {
    for (uint8_t note = 0; note < numNotesPerOctave; note++)    // 0 to 11
    {
      if (frequency >= notes[octave][note].lowestFrequency && frequency < notes[octave][note].highestFrequency)
      {
        note_found = note;
      }
    }
  }*/

  return note_found;
}



// **** Display results ****************************************

//#define MAX_CHARS 25

#define CHAR_ZERO  0
#define CHAR_ONE   1
#define CHAR_TWO   2
#define CHAR_THREE 3
#define CHAR_FOUR  4
#define CHAR_FIVE  5
#define CHAR_SIX   6
#define CHAR_SEVEN 7
#define CHAR_EIGHT 8
#define CHAR_NINE  9
#define CHAR_OFF   10
#define CHAR_C     11
#define CHAR_CS    12
#define CHAR_D     13
#define CHAR_DS    14
#define CHAR_E     15
#define CHAR_F     16
#define CHAR_FS    17
#define CHAR_G     18
#define CHAR_GS    19
#define CHAR_A     20
#define CHAR_AS    21
#define CHAR_B     22
#define CHAR_NEG   23

#define MAX_CHARS 24

// first note (CHAR_C) is at index:
#define NOTE_OFFSET 11

/*
 * Seven-Segment Display: segment names
 *    A
 *  F   B
 *    G
 *  E   C
 *    D   dp
 */

const uint8_t led_digits[] = {
  //EDC.BAFG

  0b11101110,     // 0 CHAR_ZERO
  0b00101000,     // 1 CHAR_ONE
  0b11001101,     // 2 CHAR_TWO
  0b01101101,     // 3 CHAR_THREE
  0b00101011,     // 4 CHAR_FOUR
  0b01100111,     // 5 CHAR_FIVE
  0b11100111,     // 6 CHAR_SIX
  0b00101100,     // 7 CHAR_SEVEN
  0b11101111,     // 8 CHAR_EIGHT
  0b01101111,     // 9 CHAR_NINE

  0b00000000,     // all off CHAR_OFF

  0b11000110,     // C  CHAR_C
  0b11010110,     // C# CHAR_CS
  0b11101001,     // D  CHAR_D
  0b11111001,     // D# CHAR_DS
  0b11000111,     // E  CHAR_E
  0b10000111,     // F  CHAR_F
  0b10010111,     // F# CHAR_FS
  #ifdef LARGE_G
    0b11100110,   // G  CHAR_G  ("G" shaped)
    0b11110110,   // G# CHAR_GS ("G" shaped)
  #else
    0b01101111,   // G  CHAR_G  ("g" shaped)
    0b01111111,   // G# CHAR_GS ("g" shaped)
  #endif
  0b10101111,     // A  CHAR_A
  0b10111111,     // A# CHAR_AS
  #ifdef LARGE_B
    0b11101111,   // B  CHAR_B ("8" shaped)
  #else
    0b11100011,   // B  CHAR_B ("b" shaped)
  #endif

  0b00000001      // - CHAR_NEG
};

// definitions for the 3 target LEDs
#define LED_FLAT    4
#define LED_IN_TUNE 2
#define LED_SHARP   1
#define LED_NONE    0
/*
void test_digits(int delayTime)
{
  setLeds(LED_FLAT,    led_digits[CHAR_OFF]);
  delay(delayTime);
  setLeds(LED_IN_TUNE, led_digits[CHAR_OFF]);
  delay(delayTime);
  setLeds(LED_SHARP,   led_digits[CHAR_OFF]);
  delay(delayTime);
  
  for (uint8_t i = 0; i < MAX_CHARS; i++)
  {
    setLeds(0, led_digits[i]);
    delay(delayTime);
  }
}
*/



void display_digit(uint8_t digit)
// digit describes how to draw the character on the 7-segment display
// it's a byte, and each bit is a segment, where 0 = segment is OFF, 1 = segment is ON
{
  // Decode 7-segment pattern and switch on/off the leds.
  #ifdef COMMON_ANODE
    digit = ~digit;   // invert pattern when COMMON_ANODE display is used
  #endif

  //                                                         mask
  digitalWrite(LEDE,  digit & (1 << 7) ? HIGH : LOW);     // 10000000
  digitalWrite(LEDD,  digit & (1 << 6) ? HIGH : LOW);     // 01000000
  digitalWrite(LEDC,  digit & (1 << 5) ? HIGH : LOW);     // 00100000
  digitalWrite(LEDDP, digit & (1 << 4) ? HIGH : LOW);     // 00010000
  digitalWrite(LEDB,  digit & (1 << 3) ? HIGH : LOW);     // 00001000
  digitalWrite(LEDA,  digit & (1 << 2) ? HIGH : LOW);     // 00000100
  digitalWrite(LEDF,  digit & (1 << 1) ? HIGH : LOW);     // 00000010
  digitalWrite(LEDG,  digit & (1 << 0) ? HIGH : LOW);     // 00000001
}

void display_LEDs(uint8_t LEDs)
// LEDs contains the 3 bits indicating the 3 "target" LEDs
// bit 0 is "sharp", bit 1 is "in-tune" and bit 2 is "flat"
{
  digitalWrite(LED_FLAT_PIN,    LEDs & (1 << 2) ? HIGH : LOW);  // flat
  digitalWrite(LED_IN_TUNE_PIN,  LEDs & (1 << 1) ? HIGH : LOW);  // in-tune
  digitalWrite(LED_SHARP_PIN,   LEDs & (1 << 0) ? HIGH : LOW);  // sharp
}

void display_note(uint8_t note)
{
  switch (note)
  {
    case 0xFF:
      display_digit(led_digits[CHAR_NEG]);
      break;
    default:
      display_digit(led_digits[note + NOTE_OFFSET]);
      break;
  }
}

void display_tuning(uint8_t tuning)
{
  switch (tuning)
  {
    case 0xFF:
      display_LEDs(0);
      break;
    case TUNING_FLAT:
      display_LEDs(LED_FLAT);
      break;
    case TUNING_IN_TUNE:
      display_LEDs(LED_IN_TUNE);
      break;
    case TUNING_SHARP:
      display_LEDs(LED_SHARP);
      break;
    default:
      display_LEDs(0);
      break;
  }
}



// **** Setup ******************************************

void setup_adc(void)
{
  cli(); // Disable interrupts.

  // Set up continuous sampling of analog pin 0.

  // Clear ADCSRA and ADCSRB registers.
  ADCSRA = 0;
  ADCSRB = 0;

  ADMUX |= (1 << REFS0); // Set reference voltage.
  ADMUX |= (1 << ADLAR); // Left align the ADC value - so we can read highest 8 bits from ADCH register only

  ADCSRA |= (1 << ADPS2) | (1 << ADPS0); // Set ADC clock with 32 prescaler -> 16mHz / 32 = 500kHz.
  ADCSRA |= (1 << ADATE); // Enable auto trigger.
  ADCSRA |= (1 << ADIE);  // Enable interrupts when measurement complete.
  ADCSRA |= (1 << ADEN);  // Enable ADC.
  ADCSRA |= (1 << ADSC);  // Start ADC measurements.

  sei(); // Enable interrupts.
}

void setup() {
  // target LEDs
  pinMode(LED_FLAT_PIN,   OUTPUT);  // flat
  pinMode(LED_IN_TUNE_PIN, OUTPUT);  // in-tune
  pinMode(LED_SHARP_PIN,  OUTPUT);  // sharp

  // 7-segment display
  pinMode(LEDE,  OUTPUT);
  pinMode(LEDD,  OUTPUT);
  pinMode(LEDC,  OUTPUT);
  pinMode(LEDDP, OUTPUT);
  pinMode(LEDB,  OUTPUT);
  pinMode(LEDA,  OUTPUT);
  pinMode(LEDF,  OUTPUT);
  pinMode(LEDG,  OUTPUT);


  #ifdef DEBUG
    Serial.begin(115200);
  
    //test_digits(200);  // run some LEDs showcase
  
    Serial.println(F("\nTune-O-Matic"));
  #endif
  
  setup_all_frequencies();

  setup_adc();
}

//#define TEST_FIND_NOTE
#ifdef TEST_FIND_NOTE
  uint16_t test_increment = 0;
#endif

// **** Main loop ******************************************
void loop()
{

  uint16_t frequency = 0;
  frequency = (TIMER_RATE_10 * 100) / (averaged_period >> 4); // Timer rate with an extra zero/period.

  #ifdef TEST_FIND_NOTE
    // overrides frequency calculation
    frequency = test_increment;
    test_increment += 10;
    if (test_increment > 25000) { test_increment = 0; }
  #endif

  uint8_t octave = 0xFF;
  uint8_t tuning = 0xFF;
  
  uint8_t note_found = find_note(frequency, octave, tuning);

  display_note(note_found);
  display_tuning(tuning);

  #ifdef DEBUG
    Serial.print(averaged_period >> 4);
    Serial.print(F(" "));
  
    Serial.print(F(" octave: "));
    Serial.print(octave);
    Serial.print(F(" "));
    Serial.print(F(" note: "));
    Serial.print(note_found);
    Serial.print(F(" "));
    Serial.print(F(" tuning: "));
    Serial.write(tuning);
    Serial.print(F(" "));
    
    Serial.print(frequency);
    Serial.println(F(" dHz"));
  #endif
}
