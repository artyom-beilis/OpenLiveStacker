# OpenLiveStacker - live stacking application

OpenLiveStacker is an application for Electronically Assisted Astronomy (EAA) that uses an external camera for imaging and performs live stacking. It runs on Android and in Linux environments (including Linux Subsystem for Windows).

## Status

- Beta, runs on Linux and Android.
- Platforms: Linux, PI and  Android, Windows services for Linux are working as well
- Cameras:
    - Supported:
        - ASI ZWO
        - ToupTek and Meade (based on ToupTek)
        - UVC Based like webcam, sv105
        - DSLR/DSLM support using gphoto2
        - Indi CCD - remote camera
        - Android Camera
        - Watch directoy for files (universal integration with ekos, Linux only)
        - Sim - simulation for development
    - Linux Only:
        - Indigo CCD
    - Planned to be supported:
        - SVBony - linux only (android SDK was not released yet by SVBony)
        - QHY - linux and Android
- Featues:
    - Live Stacking:
        - Deep Space Objects
        - Planetary
        - Non-Tracking - point and observe
    - Auto/Manual stretch
    - Mount control via indi (both Android and Linux)
    - Plate solving, synchronization with mount
    - Gradient removal, satellite removal, hot pixel correction
    - Calibration frames: darks, flats, dark-flats

## Using on Android

Install from [Google Play](https://play.google.com/store/apps/details?id=org.openlivestacker&hl=en) or APK file directly from release page

Please refer to <https://github.com/artyom-beilis/OpenLiveStacker/wiki/Open-Live-Stacker-Manual> for use instructions

## Using on Linux

For Linux AMD64 arch you can just download tar that contains everything, untar and run bin/ols_gui

## How to combine OpenLiveStacker in your EAA rig

![RIG](https://github.com/artyom-beilis/OpenLiveStacker/blob/main/docs/connection-charts.png)

## Building

### Build Requirements

3rd party libraries needed:

- libtiff
- cppcms 2.0 beta
- OpenCV >= 3.2 core, imgproc, imgcodecs
- C++11 enabled compiler
- Per camera requirement:
    - ASI ZWO SDK for ASI support
    - ToupTek SDK for support of ToupTek, Meade cameras
    - for UVC cameras libuvc and libusb
    - for DSLR (under development) libgphoto2
    - libraw for watch directory driver support of dng/raw files
    - libcfitsio for watch directory driver support of raw/mono fits files and for indi or indigo cameras.
    - For indigo cameras indigo development files (see below)
    - For indi cameras indi development packages (see below)
- For mount support indi development packages.

Here the packages you can install on apt based distributions

    apt-get install libgphoto2-dev git python3 cmake libuvc-dev libtiff-dev libpcre3-dev libcurl4-openssl-dev zlib1g-dev libraw-dev libopencv-dev libopencv-imgcodecs-dev libopencv-imgproc-dev build-essential libpcre3-dev zlib1g-dev libcfitsio-dev

For Indi support camera and mount:

- For installation refer to [the official documentation](https://indilib.org/download.html), specifically `libindi-dev` is required on apt based distributions
- Additionally libnova-dev is required

For ingigo camera support

- Refer to [this documentation](https://www.indigo-astronomy.org/downloads.html) and `indigo` package is required on apt based distributions

For installing CppCMS please refer to: <http://cppcms.com/wikipp/en/page/cppcms_1x_build>, in the nutshell

    git clone https://github.com/artyom-beilis/cppcms
    cd cppcms
    mkdir build
    cd build
    cmake -DDISABLE_STATIC=ON -DCMAKE_INSTALL_PREFIX=/usr ..
    make
    make install
    

### Building

Clone repo:

    git clone --recurse-submodules https://github.com/artyom-beilis/OpenLiveStacker

cd OpenLiveStacker and run

    mkdir build
    cd build
    cmake ..
    make
    

For 3rd part libraries or non-standard installations like SDK pass `-DCMAKE_INCLUDE_PATH=/path/to/includes` and `-DCMAKE_LIBRARY_PATH=/path/to/libs` parameters to cmake. For example if you have your ASI SDK here `/home/me/ASI_linux_mac_SDK_V1.28/` and you build on `x86_64` architecture use following cmake command:

    cmake -DCMAKE_INCLUDE_PATH=/home/me/ASI_linux_mac_SDK_V1.28/include -DCMAKE_LIBRARY_PATH=/home/me/ASI_linux_mac_SDK_V1.28/lib/x64/ ..

For building for old Intel CPUs without SSE4 support add `-DNO_SSE4=OFF` to cmake.

### Running

Start service:

    ./build/ols_cmd config.json

Important parameters in config.json:

- `driver` - one of asi, uvc, sim or wdir 
- `libdir` - path to directory with drivers, for example build

Open browser and go to `http://127.0.0.1:8080/` to open UI

For more details on using OpenLiveStacker on Linux refer to: <https://github.com/artyom-beilis/OpenLiveStacker/wiki/Linux-Instructions-for-using-OpenLiveStacker>


