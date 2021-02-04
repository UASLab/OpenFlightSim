#!/bin/sh
# Start Flightgear so that it listens to the JSBSim socket
# Example: ./fgfs_JSBSim.sh UltraStick25e
# Seperately run:
#   JSBSim --realtime jsb_UltraStick25e_HIL.xml
#
#   the JSBSim script should contain:
#      <output name="localhost" type="FLIGHTGEAR" protocol="UDP" port="59500" rate="50"/>

DIR=$(dirname $0)
MODEL=$1

nice fgfs \
    --fg-aircraft="$DIR/aircraft" \
    --aircraft="$MODEL" \
    --fdm=null \
    --native-fdm=socket,in,50,,59500,udp \
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
    --enable-terrasync \

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
