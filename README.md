# TurbineCV

This is a simple MIDI-to-CV converter, mainly designed for wind controllers with
high-resolution breath output, but can be used with any MIDI controller that has one or
more control-channel outputs. It can read up to four CC messages, as well as velocity and
"note pitch" (1V/oct).

It is designed to run on most low-end Arduino hardware. The default pinout configuration
is suitable for an Arduino Pro Micro, but almost any ATMega 328 or 32U4-based board should
work. However, a 32U4 one is easier as the hardware serial pins are not used by USB. It can
also be used on more powerful Arduinos such as the Mega2560 as well as Teensy.

TurbineCV has six "output slots" for analog values. By default, the first four are assigned
to CC messages. The last two are used for note pitch and velocity.

## Hardware

For MIDI in/out, either use a MIDI shield or the regular setup of an opto-coupler, diode
and a couple of resistors to the serial port of the Arduino.

Output is done via three MCP4822 DACs connected via SPI. The output from those can then be
connected straight to the CV input of a synthesizer, but it is recommended to do it via a
set of op-amps. Most generic ones will do, I've used MC1458P or TL072.


## Settings

Multiple settings can be done compile-time.

### MIDI channel

Change the `MIDI_CHANNEL` to be whatever channel you want to listen to. 0 means "all channels",
default is channel 1.

### Pitch bend range

`PITCH_BEND_RANGE` controls how far "full pitch bend" will go (up or down).

### CC list

To change which Continuous Controller (CC) parameters are assigned to which outputs,
change the values in the `cc_list`.

### High-resolution CC

TurbineCV will automatically detect and handle high-resolution CC messages properly.
Normally these are sent as a CC message 32 above the low-resolution part (for example,
for a breath controller using CC 2, high-resolution value is on CC 34). In most cases, if
your MIDI controller does not send CC values, you can leave this as is. However, if the
high-resolution value is sent some other way, you can reconfigure that here. Also, if
the high-resolution CC message for something else, it can be disabled. Do this by changing
the values in `hires_cc`.

### Chip select pins

Set `cs_pins` to suit your hardware. CC values 0 and 1 go to first DAC, 2 and 3 to second,
4 and 5 to third.

### Gate and trigger pins

Gate and trigger is sent out to two digital output pins. You may want to also run these via
output buffers (such as opamps). The pins are defined in `GATE_PIN` and `TRIG_PIN`.

### Trigger width

The trigger pulse is on at the same time as the playing note is. The `TRIGGER_WIDTH` setting
controls how wide (long) that pulse is. This needds to be less than the minimum time between two
notes, or it will not be registered as two separate pulses.

### DAC gain

The MCP4822 can be set to two different gain levels. Low gain is 0-2.048V, high gain
is 0-4.096V. You can select the gain in `dac_gain`, per output channel.
If you have an adjustable-gain op-amp on the output, low gain might be more useful as it
will leave more room for adjustment (since the common way to do a non-inverting amplifier
cannot have gain less than 1 without an attenuator). In some cases, high gain
might be more useful, such as when connecting the DAC straight to the synthesizer.

### CV invert
Normally output is low voltage for CC=0 and high for CC=127. If you want to invert this
(high voltage for CC=0, low for CC=127), you can do that per output channel in `cv_invert`.
