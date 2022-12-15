This is the data flow (up down)


            CAMERA    Control
            (frames)  (Control Stacking)
              |         /
              |    ----
              |   /
            Video Generator: bayer/yuv2rgb, rgb2jpeg, jpeg2rgb when stacking
              | \
              |  - Save Debug Images
              |
            preprocessor: rgb2float, darks/flats, derotate
              |
            Stacker: forward live video/stacked frames, stack images, stretch
              |
              |- Live Video: send live video for stacked/non-stacked image
              |- Preview Video: send downscaled with max pooling live video to show while stacking
              |- Stats: send stacking stats/status to web client
