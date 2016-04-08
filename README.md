# SousVido
It's a sous vide cooker, in Arduino!

Many of the concepts were based on Adafruit's [Sous Viduino](https://github.com/adafruit/Sous_Viduino). Aside from structuring the code differently from the ground up, this implementation also departs pretty wildly in terms of both hardware and software assumptions.

Major differences:
 - Uses a potentiometer knob (rather than a software UI) to adjust the set point. I think this simplifies using the cooker significantly.
 - Relies entirely on the autotune libary to set the PID Controller's K parameters, rather than either storing them or allowing the user to select them. This is partially for a more simplified interface, but also makes a clear choice in favor of unknown/inconsistent heating elements and container volumes.
 - Adds "PRIMING" and "RESTING" states, to get the system in the right ballpark before passing control over to the PID
  - With the right settings, this system can become a naive thermostat-like controller with no PID
 - Adds a Pause button and "PAUSED" state, useful for readjusting the food/heaters/pump without breaking things
 - Adds a Ready LED, which turns on when the system is in "RUNNING" state
 - Adds a Ready Buzzer with little melodies that play when entering/exiting "RUNNING" state
