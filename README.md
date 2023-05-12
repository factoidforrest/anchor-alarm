# Anchor Alarm

This is a DIY anchor alarm. It is designed to run on the Esp32.


## Hardware

* Mainboard: I ran it on the Esp32-c3 LilyGo OI plus, which is a compact form factor esp32 with an integrated battery slot. It could easily be adapted to others.

* GPS: I used a compact neo-8m GPS but this could be adapted to virtually any GPS with no code change. Anything supported by tinygpsplus

* Display 1306 .96in display



* A buzzer. I used a magnetic, non self-oscillating buzzer. Small ones are better because they don't overpower the board and require other electronics like flyback diodes. You could also use a breakout board based buzzer, with those diodes incorporated. That would probably be safer for the esp32 and perhaps even louder. 


## 3d Printed Case

If you match the above hardware including the mainboard, you can use my 3d printed case. TODO: ADD LINK TO CASE

## Software

See src/main.cpp for the sketch. Other files are simply tests/ scratchpads I used for debugging. There's also a lot of extra files in here because this was forked from the LilyGo sample project, since this board is nonstandard and needs a lot of custom stuff for the firmware to compile. Standard esp32s wont need that. 

### Compilation

This project is compiled by platformio. It will handle all library fetching. Change the board target if you're using a different esp32 version or 8266. 