# Anchor Alarm

This is a DIY anchor alarm. An anchor alarm is a device that beeps if you take it further than the set distance from where you turned it on, to tell you if your anchor is dragging and you need to wake up before you run aground.

It is designed to run on the Esp32.

![top down pic of the thing in a case](/schematic/topdownpic.jpg)

## Hardware

* Mainboard: I ran it on the Esp32-c3 LilyGo OI plus, which is a compact form factor esp32 with an integrated battery slot and USB-C charging. This project could easily be adapted to other boards.

* GPS: I used a compact neo-8m GPS but this could be adapted to virtually any GPS with no code change. Anything supported by tinygpsplus

* Display 1306 .96in display

* Rotary encoder: Just about any with a button will do. 

* A buzzer. I used a magnetic, non self-oscillating buzzer. Small ones are better because they don't overpower the board and require other electronics like flyback diodes. You could also use a breakout board based buzzer, with those diodes incorporated. That would probably be safer for the esp32 and perhaps even louder. The small buzzer I used is just loud enough to be useful, but could easily be drowned out by, say, a loud fan.

![wiring pic](/schematic/wiring.jpg)


## 3d Printed Case

If you match the above hardware including the mainboard, you can use my 3d printed case. STL files are in the repo root.

![case pic](/schematic/sideonpic.jpg)

## Software
Software has over-the-air update support: it creates a public wifi network for five minutes after boot. Hit 192.168.4.1/update to load new .bin firmware fil.

The device will alarm if it runs out of battery, loses the lock for over 10 seconds, or loses communications with the GPS.  

Only token attempts have been made to improve battery life.  It could probably be improved significantly by putting the GPS/esp32 to sleep for appropriate intervals, but that's easier said than done. It appears to only last about 5 hours at present, and so should be plugged in overnight to your phone charger on the boat.


See src/main.cpp for the sketch. Other files are simply tests/ scratchpads I used for debugging. There's also a lot of extra files in here because this was forked from the LilyGo sample project, since this board is nonstandard and needs a lot of custom stuff for the firmware to compile. Standard esp32s wont need that, you can just take the main.cpp sketch file.

### Compilation

This project is compiled by platformio. It will handle all library fetching. Change the board target if you're using a different esp32 version or 8266. 
