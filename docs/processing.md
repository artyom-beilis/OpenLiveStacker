This is the data flow (up down)


            CAMERA    Control (app)
            (frames)  (Control Stacking)
              |         /
              |    ----
              |   /
            Video Generator: bayer/yuv2rgb, rgb2jpeg, jpeg2rgb when stacking
              | \                       \                               \
              |  - Save Debug Images     - Live Video Generator (app)     - Save frame for Plate Solving
              |                                                           /
            preprocessor: rgb2float, darks/flats, derotate               /
              |                                                         /
            Stacker: stack and stretch images -------------------------/
              |
              |- Stats: send stacking stats/status to web client
              |
              |- Stacked Video: send video of stacked images (app)
