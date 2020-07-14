﻿**Introduction.**
 I became a licensed radio amateur when I retired some 10 years ago, my call is ON4JRT.
 Alas, moving to an apartment building a few years ago I lost the possibility to install a decent antenna. Even if I had the permission to install an antenna on the roof, the QRM generated by the building itself and the surrounding town would probably have made a clean reception almost impossible.
 A friend, who lives in the country side and to whom I am eternally grateful, came to my rescue and allowed me to install a remote transceiver and a modest antenna in his garden.

There are several remote transceiver projects described on the internet. Whilst they all are certainly more sophisticated and more performing than the rather simple system presented here, they are also much more demanding in resources (space, power, money, ...). I wanted a small stand alone system that would be self sustaining (save for an internet connection), not too conspicuous and not too expensive.
 Hence this project, which eventually was successful after a few trials. Its aim is to provide me with an remotely located transceiver for my own use, it is by no means a &quot;public&quot; transceiver available to other OM&#39;s.
 Like all engineering endeavor it consist of a series of choices and compromises, so don&#39;t shoot the pianist.

**The concept.**
 The remote transceiver system consists of several parts: A transceiver (obviously), a HF antenna, a battery, a small solar panel, a controller with a WiFi transceiver and a controller for the solar panel. Except for the HF antenna and the solar panel everything is enclosed in a small plastic housing of about 5 liter in volume.
 The remote transceiver communicates (i.e. the bidirectional data flows for sound, CAT and control) over the internet with a home device. This home device contains a simple user interface allowing some control over the remote device. Sound input and output is also available on this home device, as is a CAT interface. If I were a better PC/Posix software programmer I would probably gotten rid of the home device. Having the home device as a physical thing eases however the development (at least for me) while not adding too much to the cost.

**The transceiver.**
 After some search, I came across the µBIT-X (&quot;[https://www.hfsignals.com/index.php/ubitx/](https://www.hfsignals.com/index.php/ubitx/)&quot;) sold as a semi assembled kit developed by Ashhar Farhan (VU2ESE), now in its 6th version (I got the previous version for this project). It is a simple but rather well designed multi band HF QRP transceiver based on a double super architecture and bidirectional amplifiers (it uses almost all the building blocks for reception as well as for transmission). It is well documented, has a large fan club and most of the suggested improvements are officially endorsed.
 Coming out of the box, it needs to be calibrated and tuned (a bit). Apart from the CAT baud rate I didn&#39;t feel the need to modify anything. Obviously I didn&#39;t install the LCD or the rotary encoder (since nobody would look/touch at it), all control is performed via the CAT connection over the internet from the home device, and even then, very little of the CAT commands are really used.

**The µBIT-X controller, hardware.**

Beginning with the power supply: a DC/DC controller reduces the battery voltage to 4V, from there three linear regulators make the necessary 3.3V digital, 3.3V analog and 1.8V respectively.

An ESP32-WROOM module is the core of the controller, this awesome microprocessor includes the WiFi transceiver, performs all relevant IP chores and runs all the application software.
 It then interfaces a WM8731 codec by a I2Sbus for the digital sound and a I2Cbus for its control. This codec is a straight forward 16bit stereo ADC and DAC combination, it is programmed in this case for a sampling rate of 8kHz, yielding a sound bandwidth of 3400Hz, its dynamic range exceeds 90dB. The analog sound input and output from the codec goes through a set of isolating 600 Ohm 1:1 audio transformers and trimpots to the µBIT-X sound stages.
 The first UART of the ESP32 is exclusively used for programming and debugging.

The second UART interfaces the CAT of the µBIT-X including a PTT line. This interface is isolated with a set of optocouplers, limiting the baud rate to 4800. Note that the RD and TD lines of this interface actually bypass the USB-UART chip on the Arduino Nano board of the µBIT-X, luckily a series resistor was placed in the RD line of this Arduino Nano, hence no track cutting was nescessary.
 The third UART interfaces the Solar Controller board, it is also fit with optocouplers.
 At last the board is equipped with a PMOS switch capable of more than 2A to power the µBIT-X itself when required.

An additional small board comprising a simple tandem match type VSWR bridge is mounted directly on the antenna input/output of the µBIT-X. The detected forward and reflected signals are fed into the ADC inputs of the ESP32. With the help of some math and calibration, it gives a rather good indication of the HF power and VSWR during transmit.

**The µBIT-X controller, software.**

The software was build on the Arduino IDE (version 1.8.12) with the ESP32 plug in and libraries (version 1.0.4). It took several months of writing and careful testing, having the home device nearby was a great help.
 The software was not especially designed to be universal or portable. The several parameters like the IP addresses, URL&#39;s, etc. are &quot;hard coded&quot; in the software, no attempt was made for now to store them in a file structure or other form of non volatile memory. Hence there is no way to change them without recompiling the entire software.
 The software includes the interesting possibility of being updated &quot;over the air&quot;, i.e. via the WiFi as per the method recommended by Espressif. This was tested but not yet used, an additional port forwarding on the remote router is needed tor this.

The setup part of the software starts with the initialization of the several internal and external peripherals. After that a WiFi connection with one of the friendly WiFi AP&#39;s is established, that done the µBIT-X Controller will synchronizes with an NTP server, checking in at a DDNS server and send a short e-mail as a sign of live. Once the setup is done two endless loops will take care of business until powered off.
 One of the loops is dedicated to the audio interface, seeing to it that the digital sound samples flow in and out the codec through the I2Sbus at a constant rate of 8kHz.
 The other loop will handle the incoming and outgoing UDP packets. Outgoing packets are sent only when incoming UDP packets are present, i.e. only during a connection between the µBIT-X controller and the home device.

The data in each packet is split into three separate flows, the biggest one being the sound, a smaller one for the CAT going directly to/from the µBIT-X and the last one for the housekeeping of the µBIT-X Controller itself and the Solar Controller.

During a connection there is a UDP flow of 25 packets per second in each direction. Each packet has 660 bytes is build as follows:
 - bytes 0...639 : 320 16bit sound samples, 8000 samples/s

- byte 640...641 : packet count and control byte, including the PTT bit

- byte 642...646 : CAT bytes, max 5 to/from the µBIT-X

- byte 647...659 : CTL bytes, max 13 to/from the Solar Controller
 The sound samples of the incoming packets are stored in a circular buffer of 1s. Currently there is no provision made to cope with lost packets, if the circular buffer runs out, silence is send to the codec until its filled again for at least a quarter. Packet loss has not been a problem so far, typically no more than one packet per thousand gets lost.
 Likewise, differential drift of the sample clock, the 8kHz sampling rates at send and receive sides are not exactly equal, is simply ignored, though I may change this in future.

A few words about the IP addressing. In normal circumstances both the home device and µBIT-X Controller are connected to their respective WiFI AP&#39;s that are typically part of a modem router devices provided by Internet Access operators. The IP addresses of these devices are variable over time as the operators see fit. Hence the use of a DDNS server, the free service &quot;www.duckdns.org&quot; was chosen here. Both sides of the link are then known by their URL&#39;s i.e. &quot;remote\_name.duckdns.org&quot; and &quot;home\_name.duckdns.org&quot; respectively.
 That is one problem solved, still the packets need to pass the modem routers and be directed to the devices. This is done by setting a port forwarding for the appropriate port number in the modem router at both sides.
 Most modem routers have the tendency to assign a new IP address to the devices each time they connect. In order to avoid that, fixed IP addresses are organized for each device, hence the port forwarding can be set once and for all. The fixed IP addresses should carefully be chosen in order not to disturb the other IP stations on the local area (i.e. choosing one outside of the DHCP range of the modem).
 A more elegant solution to avoid the fixed IP addresses and the port forwarding settings in the modem routers would be the use of the UPnP method. There are good libraries for this method, e.g. &quot;[https://github.com/ofekp/TinyUPnP](https://github.com/ofekp/TinyUPnP)&quot;, with which I played a bit. It probably works but I was not able to test it due to some issues with my own modem router, so I stick with the port forwarding and fixed IP addresses for now.

**The solar panel controller.**
 The µBIT-X controller board (see further) takes about 38mA during normal operation (the µBIT-X itself being turned off), this was deemed too much to have it continuously running. Hence a scheme was preferred where the remote transceiver would become available through the internet for a few hours everyday at some predetermined time, e.g. from 8 o&#39;clock in the evening till midnight. This scheme works for me, though it may not be the case for everybody.

For this alarm clock/timer function a 24 hour real time clock was implemented in software. The power on time, the power off time and also the current time are settable remotely once a connection with the home device has been made.
 Besides of the above function the solar controller board is also supervising the charge of the battery. This function is kept extremely simple: when the battery voltage rises higher than 12.3V a PMOS switch is opened prohibiting the further charging of the battery from the solar panel. Likewise, if the battery voltage falls below 9.3V an other PMOS switch will cut the load (i.e. the rest of the system) in order to protect the battery from deep discharge damage. As the solar panel itself behaves very much like a current source and yields at most about 0.6A, no attempt is made to limit the charging current, it remains well below the maximum charging current of the Li-Ion battery.

This little board was designed some four years ago, it is built around the ATtiny84 chip and is furthermore connected via a serial port to the µBIT-X controller for housekeeping, such as the above mentioned timings, the battery voltage and the temperature inside the box. It takes a mere 3mA (continuously) from the battery.

**The battery.**
 Nothing very much is to be said about the battery, it is just an assembly of 18 Li-Ion cells of the 18650 format, 6 in parallel and 3 in series, yielding a nominal voltage of 11.1V and a capacity of 13Ah.
 Nothing is foreseen to perform load balancing, as is typical for Li-Ion batteries. After three &quot;campaigns&quot; in the field of 6 months each, no significant voltage drift was noted on the individual cell groups. This may be due to the rather gentle use of this battery, it is used mostly when nearly full (close to 12V), charged with no more than 0.6A (limited by the solar panel) and discharged with no more than 1.5A (µBIT-X transmitting) and the latter only during short periods.
 There is some concern about the temperature range of the the Li-Ion batteries: 0...40°C, which can easily be overreached during cold winters and hot summers. A thin Aluminum sheet is foreseen behind the south side of the housing to provide some shade for it.

**The housing.**
 A regular fridge/kitchen type PE box with a watertight lid of roughly 300\*200\*100mm (ca. 5 liter) was chosen as a housing for the entire remote system. An unorthodox choice, but there were never any problems with rain or weather during the accumulated 18 plus months of outdoor live so far.
 The box is traversed by two cables, one for the HF antenna (RG 58 coax) and one for the solar panel (round flexible twin electricity cable), both foreseen with pressure glands.
 Two &quot;feet&quot; are furthermore foreseen to enable the box to stand upright with its 200mm side vertical.
 The rear part of the box is reserved for the battery, the front part for the WiFi antenna (a double biquad), the in between space is filled with the µBIT-X, the µBIT-X controller and the solar controller. It was a bit of a shoe horn exercise, but nobody did complain...
 A few pouches of silica gel drying agent were inserted before the final closure of the lid.

**The solar panel.**
 The solar panel looks a bit like a picture frame, a slim Aluminum frame holds two sheets of glass between which 35 poly-crystalline photo voltaic cells are mounted in series. Its dimension is 330\*290mm. It is rated 10W by the manufacturer, if exposed perpendicular to the sun during a clear day it would yield about 17.5V (open circuit) and 0.6A (short circuit). It is installed in the garden more or less in an open space, near to the remote transceiver box, facing south and tilted at 45°.
 Theoretically, considering the statistics for my part of the world, it would yield 3.5Ah on an average summer day and 0.7Ah on an average winter day. Plenty to play with, at least during summer.

**The antenna.**
 Discretion is the keyword here, a simple 20m band wire dipole was installed horizontally between the trees in the garden. For practical reasons the resonant feed line design was chosen here, the height above the ground is about 3.5m. Alas, too low to make it a good DX antenna, but then it was that or nothing. The antenna is entirily made of RG58 cable, a nylon rope runs along with it for improved strength.
 The antenna impedance was measured and a small fixed LC circuit was installed in the remote transceiver box as an antenna tuner of sorts to ensure that the VSWR remains below 1.5.
 It was noted that the VSWR varies from 1.2 to 1.5 depending the weather.

**The home device, hardware.**
 The hardware of the little home device is very similar as that of the µBIT-X Controller. The sound is connected to the audio input/output of a PC with a 3.5mm audio jack. The incoming sound is also diverted to a small (30mm) loudspeaker to monitor the received signal.
 Note that the sound levels are kept intentionally high (i.e. around 1Veff), this in order to maximize the S/N ratio. The sound input/output towards the PC must hence be set accordingly (maximum for the output and almost nil for the input).
 The second UART is led through a UART-USB chip and is used for the CAT. The USB connection is also used for powering the home device. Further peripherals are a small OLED display (128\*64 pixels) connected to the I2Cbus (to monitor some vital parameters of the remote), a rotary encoder (to change the frequency), a potentiometer (speaker volume) and a toggle switch (to start/stop the connection with the remote).

**The home device, software.**
 The software of the little home device is also very similar to that of the µBIT-X Controller. No NTP synchronization or e-mail sending, instead the OLED display is configured during setup.
 During looping, the sending of packets is controlled by the toggle switch. Control and CAT messages coming from the remote are parsed and put on the OLED display: current frequency and mode, HF power out, VSWR, received packet count (at both ends), RSSI of the WiFi link (at both ends), battery voltage and temperature inside the housing.

A few words about the handling of the CAT messages. By design the µBIT-X emulates (at least the meaningful part of) the CAT messages of the Yeasu FT817/FT857 transceiver. This works well when the µBIT-X is connected directly to a PC, I could however not make it work when connected to through the chain of devices and the internet, most probably a question of delay.
 The following scheme was then implemented: The home device emulates in itself the above transceivers, responding immediately to satisfy the application on the PC. If something is changed (in practice the frequency or the mode) the corresponding message is send over the internet to the remote µBIT-X controller and to the µBIT-X itself. Any response of the latter is returned over the internet to the user interface of the home device.
 This scheme works well with Fldigi, however not at all with WSJT-X for reasons I have yet to understand. As for the PTT, three methods are simply used in parallel: the DTR method, the CAT method and the VOX method (implemented in software).

**Construction notes.**
The PCB&#39;s were designed with KiCad (version 5.1.6) in the form of a two layer board with generous ground planes. Most of the components are SMD though still solderable by hand.
 The board assembled in one panel were manufactured by jlcpcb.com, they offer an excellent and fast service. Care was taken to get all the boards within an area of 100\*100mm, their more or less permanent prototype discount offer.
 Most of the components were purchased at &quot;www.tme.eu&quot; and &quot;[www.r](http://www.reichelt.com/)[eichelt.com](http://www.reichelt.com/)&quot;. The more exotic components (OLED display, audio transformers, ESP32-WROOM module, WM8731A, …) were purchased at &quot;[www.aliexpress.com](http://www.aliexpress.com/)&quot;, the stuff is on the road for about 6 weeks, but the price is unbeatable.

In order to load the software into the ESP32 with the Arduino IDE, a small UART-USB module should be connected to the first UART (debug, only the RD, TD and GND lines), the boot strap should be shorted and the reset button should be pressed.
 The Solar Controlled was designed several years earlier, its software was made with AVR Studio 4. Alas, there is no Arduino IDE support for the ATtiny84.
 Finally, the housing of the home device was 3D-printed (a little box of 100\*50\*30mm), it was designed with OpenScad.

**Performance of the system as a whole.**

I am active only on the 20m band for now, the antenna being strictly monoband, and then mostly in the popular FT8 mode. With the 6...7W of output power it seems that I suffer some 10...15dB disadvantage versus other stations (difference of their S/N report versus my S/N report). This is probably more an issue of the QRP nature of the µBIT-X than the fact it is connected via the internet.
 The additional time delay due to digitization and buffering of the sound does not seem to have a significant effect, it is about 0.7s.

An other concern is the lack of cooling in the rather hermetic non metallic housing. When busy for one hour atempting QSO&#39;s in FT8, the temperature inside the housing will rise by typically 5°C. Not too bad, but then it says little about the temperature of the power FET&#39;s of the PA. Initial tests however indicated that their rather small radiators didn&#39;t became too hot even after several minutes of transmitting.

Battery drain of the remote part:
 - Solar Controller: 3mA (continuous)
 - µBIT-X Controller: 38mA (when awake)

- µBIT-X receiving: 175mA (when connected with the home device)
 - µBIT-X transmitting: 1125mA (when transmitting a single tone at 6W)

**Further improvements and other considerations.**
 Since the µBiT-X is multi band, as a further evolution of this project, I intend to have a multi band antenna (at least adding the 40m band) and some form of switchable/variable antenna tuner.

Some trick to capture the speed of rotation of the rotary encoder on the home device and to vary the frequency step accordingly would be useful. Now each step is strictly 100Hz, which is rather inconvenient. Making WSJT-X&#39;s CAT work with the home device would also be nice.
 Currently there is no provision made as for internet security, i.o.w. anybody with the right format of UDP packets can access the remote device and use it (he/she has still to know the relevant URL of course). Some scheme with an encrypted signature (e.g. HMAC style) inside each packet authenticating the home device would be an improvement.
 Though the current design is intended for the use with digimodes, it can easily be adapted for voice operations. Adding a microphone amplifier and a PTT switch input would then be required.
