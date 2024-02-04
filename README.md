# ESP8266 based, Alexa control of LED strips with rotary control knob and on/off switch
This repo contains notes on creating an ESP8266 based Arduino Alexa LED strip controller, while also providing manual control.  
<br />
When a request for under-cabinet lights becomes a custom electronics project.
<br /> 

![First Box](/assets/first_box.jpg)

## Overview of this project
The goal of this project is to control the brightness of an LED strip. The implementation provides Alexa control of the LED strip.  In addition, manual control is provided by a rotary knob.  The project is implemented using the Arduino software environment and a small box of electronics. 

Being Arduino, the code is in C++.  The code uses the [Fauxmo](https://github.com/vintlabs/fauxmoESP) library to enable device to Alexa communication.  To read the movement of the knob I used the [RotaryEncoder](https://github.com/mathertel/RotaryEncoder) library.

## Contents of this page
* [Overview of this project](#overview-of-this-project)
* [Requirements](#Requirements)
* [Theory of operation](#theory-of-operation)
  * [Brightness control](#brightness-control)
  * [Microcontroller software logic](#microcontroller-software-logic)
  * [The rotary encoder](#the-rotary-encoder)
  * [The level shifter driver circuit](#the-level-shifter-driver-circuit)
* [Hardware builds](#hardware-builds)
* [Electrical components](#electrical-components) 
* [Oscilloscope measurements of the device](#oscilloscope-measurements-of-the-device)
* [Potential enhancements](#potential-enhancements)

## Requirements
* The user will be able to:
   * Turn an LED strip on and off
   * Adjust the brilliance (dim the LED)
* The user will be able to make these adjustments using Alexa voice commands, the Alexa app, or manual control
* The manual control and the electronics box may be separated.  For example, a control knob can to be located conveniently, while the small electronics box can be located in an inconspicuous place.  A wired connection is acceptable.

## Theory of operation

### Brightness control
Brightness control (dimming) is done using [Pulse-width modulation](https://en.wikipedia.org/wiki/Pulse-width_modulation) (PWM).  In this case the microcontroller sends a pulse a thousand times a second.  If that pulse is 100% wide the LED strip will be fully bright.  A 50% pulse width (50% duty cycle) will be less bright. Send 1% duty cycle pulses and the LED strip will be very dim.  Zero width pulses (no pulses) shut off the LED.  So by controlling the pulse width we control the light level.

The 3.3v pulses from the microcontroller are level shifted up to ~6.5V to better drive the power Mosfet module.  This makes it easier to use other power Mosfets types in the future. [Here you can see an oscilloscope trace](/assets/scope_images/best/dimmer_driver_triple_pulse.png) of both the ~3.3V microprocessor pulse (yellow) and the ~6.5V pulse (blue).

A 1% duty cycle pulse is very narrow but as you can see in [this scope trace](/assets/scope_images/best/dimmer_driver_1per_zoom.png) the pulse (blue) reaches the Mosfet module's ~3v switching threshold quickly and acceptably.  So all good.

The 6.5V pulses are output to the power Mosfet module to deliver a 12V pulse with the higher current the LED strip needs.  This particular LED strip is fully turned on at 12v and off below ~9.3V. When the 6.5V pulse drops back to 0V, the 12V voltage on the LED strip drops rapidly to ~9.3V and then more slowly lower.
The LED strip is quite dim with a 1% duty cycle pulse. Certainly as dim as I need.  [Here is a view](/assets/scope_images/best/LED_strip_PWM_with_1uf2.png) of the pulses on the LED strip power line.

### Microcontroller software logic
The [Arduino sketch](/code/esp8266_alexa_led_control_w_encoder/esp8266_alexa_led_control_w_encoder.ino) (program) performs these functions:
* Connects to Wifi
* Communicates to Alexa over WIFI and represents itself as a Philips smart lightbulb.
* Listens for a command from Alexa
* Watches for a turn of knob (rotary encoder)
* Watches for a push on the knob (on/off).  Software debounce is performed in the code without a library.
* Makes a pulse width change on an output pin when it determined a brightness change is requested
* Communicates to Alexa the brightness level after a change

### The rotary encoder
For manual control, the user interacts with a knob that turns right and left, and also can be pushed to toggle a switch.  The knob is connected to a rotary encoder which senses rotation direction. It also serves as a toggle push button. It communicates over an [I2C](https://en.wikipedia.org/wiki/I%C2%B2C) bus created with two microcontroller pins.  The encoder requires 5 wires between it and the microcontroller.  I originally chose an I2C encoder because I was going to put the knob on the electronics box.  User testing of the prototype revealed the need for the physical knob to separate from the box.  That could have meant a redesign of the electronics since I2C spec limits the distance to about 2.5 meters.  However I tested the encoder through 10 meters of wire to the electronics box and it worked without an issue.  My distance is only about 2 meters so I had no problems using I2C to the encoder.  So I am sticking with this design since a) it works in my application and b) I already have the I2C encoder parts.

If you are going a long distance I2C is not the bus to use.  RS485 and CAN are recommended alternatives designed for better noise immunity and longer distances.

### The level shifter MOSFET driver circuit
The level shifter circuit amplifies the 3.3V to 6.5V. It is based on logic level-shifter circuits of [this type](/assets/bi_directional_level_shifter_circuit_diagram.jpg).  They work well between logic chips.  In my case I want to drive the power Mosfet module.  Due to its lower input impedance of the Mosfet module, I found I needed to bring the R2 resistor value down from the 10k Ohm value.  Using R2 = 10k Ohms, the output voltage of the level shifter was 12V. But when connected to the Mosfet module the output voltage of the level shifter circuit dropped to 3.6v.  My objective for the circuit is to raise the output to at least 5.5 volts in order assure the power Mosfet module is fully turned on.  I found that by reducing R2 to 2.2k Ohms I could level shift the pulse to ~6.5V.  Using a 2.2k Ohm R2, the current through R2 is an acceptable ~5mA.  The schematic of my final level shifter design is [here](/assets/bi_directional_level_shifter_circuit_diagram_crop.png).  This is not a common way to drive a power Mosfet but I wanted a non-inverting circuit and am intrigued by this circuit's simplicity and its clever configuration.

## Hardware builds
* [First board](/assets/first_board.jpg)
* [First box](/assets/first_box.jpg)

## Electrical components
* [5 meter 12v LED strip (non-addressable) with 12v 3A power supply](https://www.ebay.com/itm/126175699898)  $16.99.
* [Small buck converter, 12v->5v](https://www.amazon.com/gp/product/B0CDWW4XHL)   $1.30
    * An adjustable one works fine.  
    * Only supplies power to the microprocessor, the rotary encoder, and the small 2n7000 small Mosfet.  Since the current draw is small I was able to use a very small part. 
* [NodeMCU ESP8266 microcontroller](https://en.wikipedia.org/wiki/NodeMCU)   $3.16
* [KY-040 Rotary encoder module with on/off switch and I2C interface]()    $1.25
  * Module says 5v operating voltage but KY-040 rotary encodrr suports 3.3v. I powered it with 3.3v.
* Parts for the [Mosfet level shift driver circuit](/assets/bi_directional_level_shifter_circuit_diagram_crop.png) to drive the power Mosfet module. 
  < $0.70
  * [2n7000 Mosfet transistor](https://www.digikey.com/en/htmldatasheets/production/99360/0/0/1/2n7000-datasheet)
  * 2.2k 1/4W resistor
  * 10k 1/4W resistor
  * 1000uF 25v Electrolytic capacitor used to prevent a power supply dropout from a sudden change in brightness.
* Additional parts
  * MR510 3A diode for reverse polarity protection. Size larger if your LED string pulls more currently.  My LED strip pulls about 1.5 A @12v when fully on. 
* [Mosfet Switch Drive Board 0-20KHz PWM Electronic Switch](https://www.amazon.com/gp/product/B08CXB4WC)    $1.26

## Oscilloscope measurements of the device
* [Pulse trains from microcontroller (3.3V) and level shifter (6.7V)](/assets/scope_images/best/dimmer_driver_many_pulses.png)
* [Zoom in on three pulses](/assets/scope_images/best/dimmer_driver_triple_pulse.png)
* [Single 1% duty cyle pulse](/assets/scope_images/best/dimmer_driver_1per_zoom.png) 
* [Zoom in on single 1% duty cycle pulse](/assets/scope_images/best/dimmer_driver_zoom_in.png)
* [Resulting pulses on the LED line](/assets/scope_images/best/LED_strip_PWM_with_1uf2.png)
* [Zoom on ringing of 1% pulse on LED line](/assets/scope_images/best/LED_strip_PWM_with_1uf_rise_and_ring2.png)
* [Deeper Zoom on ringing of 1% pulse on LED line](/assets/scope_images/best/LED_strip_PWM_with_1uf_rise_and_ring1.png)

## Potential enhancements
To dos
* Enhance manual control so that in the case of multiple devices turning one knob will adjust all of them.
* Create a RS485 or CAN version for use where the encoder is farther from the electronics box.  Alternately, test some I2C entender chips to increase distance but keep within I2C specs.
* Add optocouplers between the microcontroller and the power Mosfet module to better protect the microcontroller from the higher voltage the power Mosfets are switching.  That will also provide the better safety needed to handle higher voltage.  An application would be controlling speed of a 24v DC motor.
