# Tune-O-Matic
Tuner for analog synths

Forked from Jos Bouten's code (https://github.com/josbouten/Tune-O-Matic)

Portions of code from:
 - Amanda Ghassaei's Instructables https://www.instructables.com/Arduino-Frequency-Detection/
 - Sam's LookMumNoComputer https://www.lookmumnocomputer.com/projects#/1222-performance-vco

The Tune-O-Matic firmware samples an analog signal, tries to detect the rising edges, measures the time between each cycle, averages the values, converts the result to a frequency, compares that frequency to a table of frequencies/notes pairs and finally displays the corresponding note, and if it's above, below or in-tune.


The code is usable for a common anode or common cathode led display. 
Follow the instructions in the code and set LED_DISPLAY_TYPE to the one you use.

There are some alternate charsets: you can choose between "b" or "B", "g" or "G". Uncomment the corresponding definition in code.

The code is compatible with the following hardwares:
 - https://www.lookmumnocomputer.com/projects#/1222-performance-vco
 - https://github.com/MyModularJourney/Tuna
 - https://www.davidhaillant.com/category/electronic-projects/utility-modules/tuner/

