# OpenLiveStacker - live stacking application

## Status

- Early, alpha, runs on Linux, Android integration soon
- Works with UVC Cameras

## Roadmap

- Platforms:
    - Implemented: Linux
    - In near future: Raspberry PI, Android, maybe windows
- Camera
    - Implemented: UVC Based like webcam, sv105
    - Future extected drivers:
        - Watch directoy for files (universal integration with ekos)
        - Android camera
        - ASI ZWO SDK (mostly for android and Linux likely as well)
        - INDI Library (for wide range of cameras)
- Switch pure JS UI to better looking one
- Integrate as Android App

## Build On Linux

3rd party libraries needed:

- libtiff
- libuvc 
- cppcms 2.0 beta
- OpenCV >= 3.2 core, imgproc, imgcodecs
- C++11 enabled compiler

Just run

    mkdir build
    cd build
    cmake ..
    make

To run:

    ./build/ols_cmd

Note since it access USB directly you either need to:

1. Run it as root
2. Run `chmod a+rw /dev/usb/NNN/MMM` where NNN and MMM are device indexes of USB camera, you can find it by running `lsusb -t`


