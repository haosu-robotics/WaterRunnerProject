Water Runner Board
==================

Known Issues
------------

* Use absolute encoders instead of incremental encoders
    * Absolute encoders output analog signal proportional to the angle and can be read by the Arduino directly via an `AnalogRead()` command.
* Vias near diodes are too close to pads, causing shorts
* Use JST ZE series connectors everywhere instead of JST NSH connectors
    * NSH connectors are too small requiring small wire that breaks easily
    * 28 gauge wire we have for the current ZE connector on the board is  
      thicker than usual and barely fits
* Do not put any circuitry above the Arduino's USB connection as it is grounded
    * A ground plane covering this area is acceptable
* Screw terminal for +7V line is not necessary
    * Use Arduino power supply instead
