The open live stacker will work over HTTP protocol, default URL is http://127.0.0.1:8080/


API:

    / - entry to UI
    /media - all media files that are stored here
    /media/js - javascript files
    /media/img - images for the web ui
    /media/data - data files 
    /media/data/calibration/ - stacked calibration frames
    /media/data/calibration/index.json - index of calibration data
    /media/data/stacked - results of live stacking
    /media/data/debug - saved intermediate data for debugging, postprocessing
    /media/data/debug/session_id/ 
    /media/data/debug/session_id/log.txt - log of all the results, meta data etc, everything needed to recreate stacking offline
    /media/data/debug/session_id/(other images) -  saved frames depedning on type
    
    /api - base URL to API access
    /api/video/live - live video stream non-processed MJPEG to show directly in browser
    /api/video/stacked - video of stacked stream MJPEG to show directly in browser
    
    
    GET /api/camera
        return json list of cameras that can be open:
        [
            {
                "id"   : number // camera no - 0..N
                "name" :  string, // camera name. to display
                "type" : string, // UVC, ASI, ASCOM, AndroidCamera, VFL 
            }
        ]
    GET /api/camera/ID - open camera, get list of supported formats/sizes/mods
        [
            {
                "format_id" : string, // format id
                "width" : INT, 
                "height": INT,
                "framerate"? : float, null // max framerate supported
                "format": string // One of "MJPEG", "YUV", "RAW" etc.
            },
        ]
    POST /api/camera/ID/stream - start stream
        {
            "format_id" : string, // format you received in the list, for example "1920x1080_mjpeg"
        }
    GET /api/camera/ID/options - list camera suppored settings
        [
            {
                "option_id" : string // option name id for example "exp", "wb"
                        // Supported set (to be extended)
                        // auto_exp - bool, auto_wb - bool
                        // exp, wb, gain, gamma, 
                "name" : string // option name human readable: "Exposure", "White Ballance"
                "type" : string // "boolean", "integer", "float", etc
                "min"  : value // min value for integer/float
                "max"  : value // max value for integer/float
                "step" : value  // step unit for integer/float
                "unit" : string // unit of the value "K" for WB, "ms" for exposure, "%" for gain etc, human readable
                "default": value // default value if applicable, null for not
            }
        ]
    POST /api/camera/ID/option/OPTION_ID - set camera settings
        { "value" : value }
    GET /api/camera/ID/option/OPTION_ID - get current camera option value
        { "value" : value }
    GET /api/camera/ID/option/all - get all camera option value
        [ { "option_id" : string, "value" : value }, ... ]
    POST /api/camera/ID/option/any - update any camera option by list
        [ { "option_id" : string, "value" : value }, ... ]
