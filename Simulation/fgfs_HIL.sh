#!/bin/sh
# Start Flightgear, optionally start HIL
# Example: ./fgfs_HIL.sh UltraStick25e
#   Once started:
#     (optionally) start the SOC code, press "=" to engage
#     (optionally) start the FG autopilot modes, press "F11" to select

# Aircraft can be: UltraStick25e, UltraStick120, Rascal110, mAEWing2, etc.

DIR=$(dirname $0)

nice fgfs \
    --fg-aircraft="$DIR/aircraft" \
    --aircraft=$1 \
    --generic=socket,out,50,192.168.7.2,6500,udp,aura-imu \
    --generic=socket,out,10,192.168.7.2,6501,udp,aura-gps \
    --generic=socket,out,50,192.168.7.2,6502,udp,aura-pilot \
    --generic=socket,in,50,,6503,udp,aura-act \
    --lat=44.725801 \
    --lon=-93.075866 \
    --heading=90 \
    --altitude=285.3 \
    --disable-real-weather-fetch \
    --wind=0@0 \
    --turbulence=0.0 \
    --disable-horizon-effect \
    --timeofday=noon \
    --disable-random-objects \
    --disable-ai-models \
    --fog-disable \
    --disable-specular-highlight \

#    --geometry=650x550 \
#    --bpp=32 \

# Start in-flight
#    --altitude=2000 \
#    --vc=46 \

# MSP Airport
#    --airport=KMSP \

# STP RC
#    --lat=45.220422 \
#    --lon=-93.145448 \
#    --heading=10 \

# Jensen Field
#    --lat=44.725777 \
#    --lon=-93.075809 \
#    --heading=90 \
#    --altitude=285.3 \
