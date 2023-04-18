The open live stacker will work over HTTP protocol, default URL is http://127.0.0.1:8080/


## Web interface :

Static files: www-data

    / - entry to UI index.html
    /media - all media files that are stored here
    /media/js - javascript files
    /media/js/jsdb.js - database of DSO index to RA/DE
    /media/img - images for the web ui

Generated files

    /media/data - data files 
    /media/data/calibration/ - stacked calibration frames
    /media/data/calibration/index.json - index of calibration data
    /media/data/stacked - results of live stacking
    /media/data/stacked/[stacking_id]_final.jpeg - final result
    /media/data/stacked/[stacking_id]_final.tiff - final unstetching/non-compressed floating point image tiff if needed
    /media/data/stacked/[stacking_id]_final.json - final result information 
    /media/data/stacked/[stacking_id]_intermediate_NN.jpeg - intermediate result
    /media/data/stacked/[stacking_id]_intermediate_NN.tiff - intermediate result unstetching/non-compressed floating point image tiff if needed
    /media/data/stacked/[stacking_id]_intermediate_NN.json - intermediate result information
    /media/data/debug - saved intermediate data for debugging, postprocessing
    /media/data/debug/stacking_id/ 
    /media/data/debug/stacking_id/log.txt - log of all the results, meta data etc, everything needed to recreate stacking offline
    /media/data/debug/stacking_id/(other images) -  saved frames depedning on type
    /media/data/plate_solving_solution.jpeg - visual plate solving solution


Calibration index:

    /media/data/calibration/index.json

    [
        {"id" : string // idenfification of frame
         "path" : string// path to file relative to calibration
         "camera" : string //camera name
         "settings" : settings // camera settings see /api/camera/ID/option/all
         "date": string // local date and time of calibration frame generation
         "fames" : integer // number of frames collected
        },
    ]

Stacked result meta data per file 

    {
         "stacking_id" : string // idenfification of frame
         "path" : string// path to file relative to calibration
         "camera" : string //camera name
         "settings" : settings // camera settings see /api/camera/ID/option/all
         "stacking" : settings of stacking setup including stretching
         "date": string // local date and time when processing started
         "date_finish": string // local date and time when processing started
         "total_fames" : integer // number of frames collected
         "stacked_fames" : integer // number of frames stacked
    },

    
## API:

Basic API layout

    /api - base URL to API access
    /api/video/live - live video stream non-processed MJPEG to show directly in browser
    /api/video/stacked - video of stacked stream MJPEG to show directly in browser
    /api/updates - live updates

    /api/camera - camera controls
    /api/stacker - stacker controls
    /api/plate_solver - plate solver controls

    
### Camera API `/api/camera`

Camera controls


    GET /api/camera
        return json list of cameras that can be open:
        [
            {
                "id"   : number // camera no - 0..N
                "name" :  string, // camera name. to display
                //"type" : string, // UVC, ASI, ASCOM, AndroidCamera, VFL 
            }
        ]
        
    POST /api/camera
        {
            "operation" : "open","close" // open or close camera
            "id" : camera id
        }
    GET /api/camera/status
        
        { "status" : "open"/ "closed" /"streaming" - current status } 

    GET /api/camera/formats - open camera, get list of supported formats/sizes/mods
        [
            {
                "format_id" : string, // format id
                "width" : INT, 
                "height": INT,
                "framerate"? : float, null // max framerate supported
                "format": string // One of "MJPEG", "YUV", "RAW" etc.
            },
        ]

    POST /api/camera/stream - start/stop stream
        {
            "op" : "start" / "stop" // start or stop the stream
            "format_id" : string, // for start stream provide format you received in the list, for example "1920x1080_mjpeg"
        }
        return { "status" : "ok"/"fail", "msg" : STRING" }

    GET /api/camera/options - list camera suppored settings
        [
            {
                "option_id" : string // option name id for example "exp", "wb"
                        // Supported set (to be extended)
                        // auto_exp - bool, auto_wb - bool
                        // exp, wb, gain, gamma, 
                "name" : string // option name human readable: "Exposure", "White Ballance"
                "type" : string // "boolean", "integer", "float", etc
                "min"  : value // min value for integer/float
                "cur"  : value // current value for integer/float
                "max"  : value // max value for integer/float
                "step" : value  // step unit for integer/float
                "default": value // default value if applicable, null for not
            }
        ]
    POST /api/camera/option/OPTION_ID - set camera settings
        { "value" : value }
        return { "status" : "ok"/"fail", "msg" : STRING" }
    GET /api/camera/option/OPTION_ID - get current camera option value
        { "value" : value }
    GET /api/camera/option/all - get all camera option value
        [ { "option_id" : string, "value" : value }, ... ]
    POST /api/camera/option/any - update any camera option by list
        [ { "option_id" : string, "value" : value }, ... ]
        return { "status" : "ok"/"fail", "msg" : STRING" }

### Stacker API

Stacking controls


    POST /api/stacker/start - start stacking

        {
            "name" : string // unique id of stacking - for saving
            "calibration": "dso"/ "calibration" // live stacking for DSO, calibration frames (no registration) 
            "target" : null or { // required for derotation
                "ra": float // RA of object in degrees
                "de": float // DE of object in degrees"
            }
            "location" : or null {  // required for derotation
                "lat": float // observers lattitude degees
                "lon": float // observers longitude degees
                "name": string or null // optional, for example M31
            },
            "source_gamma" : float, default 1.0 // gamma used for image correction
            "comment": string or null // optional comment to save with the image
            "image_flip": bool // default false - inverse rotation (for newtonian/mirror diagonal)
            "field_rotation" : bool // default false, if true target and location must be provided
            "auto_stretch" : bool // stretch , default true
            // calibration frames, id taken from /media/data/calibration/index.json
            "darks": string or null // id of dark frames 
            "flats": string or null // id of flat frames
            "bias": string or null // id of bais frame
            "save_data" : bool // default false - save intermediate data used for stacking for offline processing
        }
        return { "status" : "ok"/"fail", "msg" : STRING" }

    POST /api/stacker/control

        {
            "operation" : "pause" / "save" / "resume" / "cancel" 
            // pase and resume for manual tracking, save - finish stacking and save the result, cancel - finish stacking and discard the result
            "save_unprocessed" : bool // default false - save floating point image before stretching and converting to 8bit jpeg
        }
        return { "status" : "ok"/"fail", "msg" : STRING" }

    POST /api/stacker/stretch
        {
            "auto_stretch" : bool // true or false, works even if paused
            // if false set these for manual correction
            "stretch_low" : float 0..1 - remove low level. default 0.5
            "stretch_high" : float 0..1 - remove high range, default 0.5
            "stretch_gamma" : // gamma correction powr 0..1
        }
        return { "status" : "ok"/"fail", "msg" : STRING" }

    GET /api/stacker/status
        return { "status" : "idle"/"paused"/"stacking" }
            
### Live Updates

All updates are sent via ServerSentEvents 

Image Historgrams: 

    /api/updates/live/histogram - histogram for raw video
    /api/updates/stacked/histogram - histogram for stacked image BEFORE auto-stretch

        histogram format 
        {
            "type": "rgb"/"mono"
            // for RGB
            "R" : [ 256 floating point numbers ] 
            "G" : [ 256 floating point numbers ]
            "B" : [ 256 floating point numbers ]
            // for mono
            "Y" : [ 256 floating point numbers ]
        }
   
Stacking updates   
    
    /api/updates/stacking_status - progress updates, meta info etc.

    {
        "status" : "stacking" / "failed" / "paused" / "finished" // stacking status
        "frames" : INTEGER, // number of frames received
        "processed" : INTERGER, // number of frames processing
        "stacked": INTEGER // number of rames actually stacked (may be failure due to registration errors
        "dropped": INTEGER // number of frames dropped due to overload
        "error_message": // message in case of failed status
    }


