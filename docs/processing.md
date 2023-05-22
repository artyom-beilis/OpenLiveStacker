This is the data flow (up down)


                        CAMERA    Control (app)
                        (frames)  (Control Stacking)
                        |         /
                        |    ----
                        |   /
                        Video Generator: bayer/yuv2rgb, rgb2jpeg, jpeg2rgb when stacking
                      / |   \                                \
     Save Debug Image   |     - Live Video Generator (app)     - Save frame for Plate Solving
      |                 |                                                             /
      |  /------------- preprocessor: rgb2float, darks/flats, derotate               /
     Stats              |                                                           /
      and   ----------- Stacker: stack and stretch images -------------------------/
     Errors             |
                        |
                        |- Stacked Video: send video of stacked images (app)
