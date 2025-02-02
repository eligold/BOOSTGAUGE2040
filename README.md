<h1 align="center"> RP2040 LVGL Car OBD-II Gauge </h1>
<br>
<p align="center"><img src="doc/IMG_6330.jpg" alt="prototype view" width="34%"><img src="doc/dash.png" alt="dashboard gauge" width="38%"></p>

Programatically generate gauges on a round touchscreen with LVGL for infite flexibility in what data is displayed and how.

This was going to be a relaxed weekend afternoon project that turned into an impromptu all-nighter. I'm documenting this as much to show off the results as I am to highlight the development pitfalls and ways for others to avoid some of my own mistakes. Of course you can buy customizable aftermarket gauges but...

<p align="center"><img src="doc/rick.gif" alt="I make my own stuff" width="50%"></p>

#### Inspiration:

1. ] https://youtu.be/pbqgrv5YSf0
    - I saw this video first, it is for a custom shift knob gear select indicator. While not the same screen it definitely got the creative gears turning so to speak. It is a video from a series detailing that extremely cool project and the creator [upir](https://www.youtube.com/@upir_upir) is worth a follow if you like cars and also gadgets.
1. ] https://youtu.be/cZTx7T9uwA4 - Boost gauge using image series
1. ] https://youtu.be/A00CvNi1rzQ - Temp gauge w different series of images
1. ] https://youtu.be/i-TA3Xwja64
    - Using the meters widget from a port of lvgl v8 to programatically generate gauges, first one I stumbled upon like this. Shared no source code.
1. ] https://youtu.be/y_H7HM0oyoo
    - Another set of gauges based on static images. Doesn't benefit from the configurability of lvgl instead using only the lower level TFT_eSPI driver.
1. ] https://youtu.be/gsTP7zljSBg
    - Another project based on images with more dynamic sprite utilization.
1. ] https://youtu.be/s4_fUwhw-dI - LVGL overview
1. ] https://youtu.be/mqSe_uMpxIs - Fun animated images project
1. ] https://youtu.be/g4ZVXwt5ueA
    - Generic product exposition/sponsored content for the ESP32 variety.
1. ] https://youtu.be/HzHRJd7rihE
    - Star wars tactical display was another fun diversion. Uses another lower-level graphics library and a dumb screen but the display elements are largely generated with code. Very neat project.
1. ] https://youtu.be/jis1MC5Tm8k
    - accelerometer power save idea 🤌 and also a cool project in its own right
