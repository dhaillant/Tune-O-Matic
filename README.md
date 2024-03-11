# Tune-O-Matic
Tuner for analog synths

Forked from Jos Bouten's code (https://github.com/josbouten/Tune-O-Matic)

Portions of code from:
 - Amanda Ghassaei's Instructables https://www.instructables.com/Arduino-Frequency-Detection
 - Sam's LookMumNoComputer https://www.lookmumnocomputer.com/projects#/1222-performance-vco
 - Eric Matecki's Tune-OO-Matic https://gitlab.com/matecki_kosmo/tune-oo-matic

The Tune-O-Matic firmware samples an analog signal, tries to detect the rising edges, measures the time between each cycle, averages the values, converts the result to a frequency, compares that frequency to a table of frequencies/notes pairs and finally displays the corresponding note, and if it's above, below or in-tune.


The code is usable for a common anode or common cathode led display. 
Follow the instructions in the code and set LED_DISPLAY_TYPE to the one you use.

There are some alternate charsets: you can choose between "b" or "B", "g" or "G". Uncomment the corresponding definition in code.

The code is compatible with the following hardwares:
 - https://www.lookmumnocomputer.com/projects#/1222-performance-vco
 - https://github.com/MyModularJourney/Tuna
 - https://www.davidhaillant.com/category/electronic-projects/utility-modules/tuner/

LMNC Forum about Tune-O-Matic tuner: https://lookmumnocomputer.discourse.group/t/tune-o-matic-tuner/36

## generating frequency data
The firmwares already contains values for A4=440Hz and 6 cents deviation allowed.

Use the ruby script **freqs_ranges.rb**:

  ruby freqs_ranges.rb

Copy and paste the results in the Arduino code, in the array *frequencyTable*
By default, the script generates frequencies for A4=440Hz and 10 cents deviation.
You can tweak the results by passing the following parameters:

    -f, --frequency <frequency>      A4 reference frequency (in Hz). Default is 440 Hz
    -c, --cents <variation>          Variation allowed, in cents. Default is 10 cents.
    -h, --help                       Prints this help.


