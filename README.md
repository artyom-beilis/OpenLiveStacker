# OpenLiveStacker - live stacking application

## Status

- Early, alpha, runs on Linux, Android integration soon
- Works with UVC and ASI ZWO Cameras

## Roadmap

- Platforms:
    - Implemented: Linux and  Android (see <https://github.com/artyom-beilis/android_live_stacker/tree/web_ols>)
    - In near future: need to test Raspberry PI, maybe windows
- Camera
    - Implemented
        - UVC Based like webcam, sv105
        - ASI ZWO support - initial exposure and gain controls only
        - Sim - simulation for development
    - Future extected drivers:
        - Watch directoy for files (universal integration with ekos)
        - Android camera
        - INDI Library (for wide range of cameras)
- Switch pure JS UI to better looking one
- Integrate as Android App

## Using on Android

Install APK file that can be downloaded from Releases section, you may see some warnings (that it is build for older devices) continue anyway

Open the app. Connect ASI or UVC camera to the device. You may need USB to type-C or to microusb adapter. The device should support OTG and it needs to be enabled under settings.

Note, some devices may need external power to USB be provided

Press on `ASI Device` or `UVC Device` - you'll be asked for permission to access the device and than borwser will be opened with address <http://127.0.0.1:8080>. Where you can control the stacking app.

Stacked images and all data is stored under `DCIM/OpenLiveStacker` 

## Using on Linux

### Build On Linux

3rd party libraries needed:

- libtiff
- libuvc 
- cppcms 2.0 beta
- OpenCV >= 3.2 core, imgproc, imgcodecs
- C++11 enabled compiler
- ASI ZWO SDK (optional) for ASI support

Just run

    mkdir build
    cd build
    cmake ..
    make

### Running

Start service:

    ./build/ols_cmd DRIVER build

where `DRIVER` is either `uvc` or `asi` according to the device you want to use

Open browser and go to `http://127.0.0.1:8080/` to open UI

All the captured images and calibration frames will be stored under `data` directory on Linux

**Note:** since it access USB directly you either need:

1. Provide proper udev rules that provide access to the device
2. Run `chmod a+rw /dev/usb/NNN/MMM` where NNN and MMM are device indexes of USB camera, you can find it by running `lsusb -t`
3. Run app as root


