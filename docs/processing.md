This is the data flow (up down)


            CAMERA    Control
            (frames)  (Control Stacking)
              |         /
              |    ----
              |   /
            Video Generator: bayer/yuv2rgb, rgb2jpeg, jpeg2rgb when stacking
              | \                       \
              |  - Save Debug Images     - Live Video Generator
              |
            preprocessor: rgb2float, darks/flats, derotate
              |
            Stacker: stack images
              |
              |- Stats: send stacking stats/status to web client
              |
            PostProcessor: stretch, create_output_jpeg
              |
              |- Stacked Video: send video of stacked images
