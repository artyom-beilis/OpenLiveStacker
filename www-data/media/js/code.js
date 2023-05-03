
var global_width = 0;
var global_height = 0;
var g_prev_high = -1;
var g_prev_low = -1;
var g_stacker_status = 'idle';
var g_show_thumb_live = false;
var g_stats = null;

function parseRA(svalue)
{
    const re_float = /^(\d+\.\d+)$/;
    const re_colon = /^(\d+):(\d+)(?::(\d+(.\d+)?))?$/;
    const re_hms = /^(\d+)[hH \t]\s*(\d+)[mM'′]\s*(?:(\d+(.\d+)?)(?:s|S|′′|″|''))?$/;
    const re_space = /^(\d+)\s+(\d+)(?:\s+(\d+(.\d+)?))?$/;
    var res = svalue.match(re_float);
    var deg = null;
    if(res) {
        deg = parseFloat(res[1]);
        if(deg < 0 || deg > 360)
            return null;
        return deg;
    }
    res = svalue.match(re_colon) || svalue.match(re_hms) || svalue.match(re_space);
    if(!res)
        return null;
    var h = parseInt(res[1]);
    var m = parseInt(res[2]);
    var s = 0;
    if(res[3]) {
        s = parseFloat(res[3]);
    }
    if(h  >= 24 || m >= 60 || s >= 60) {
        return null
    }
    deg = 15 * (h + (60 * m + s) / 3600.0 );
    return deg;
}

function parseDEC(svalue)
{
    const re_float = /^([+-−]?\d+.\d+?)$/;
    const re_colon = /^([-+−]?)(\d+):(\d+)(?::(\d+(.\d+)?))?$/;
    const re_dms = /^([-+−]?)(\d+)[dD° \t]\s*(\d+)[mM'′]\s*(?:(\d+(.\d+)?)(?:s|S|″|′′|''))?$/;
    const re_space = /^([-+−]?)(\d+)\s+(\d+)(?:\s+(\d+(.\d+)?))?$/;
    var res = svalue.match(re_float)
    if(res) {
        var deg = parseFloat(res[1])
        if(deg < -90 || deg > 90)
            return null;
        return deg;
    }
    res = svalue.match(re_colon) || svalue.match(re_dms) || svalue.match(re_space);
    if(!res)
        return null;
    var sig = (res[1] == '-' || res[1] == '−')? -1.0 : 1.0;
    var d = parseInt(res[2]);
    var m = parseInt(res[3]);
    var s = 0;
    if(res[4]) {
        s = parseFloat(res[3]);
    }
    if(d  > 90 || m >= 60 || s >= 60)
        return null;
    return sig*(d + (60 * m + s) / 3600.0);
}

function restCall(method,url,request,on_response,on_error = alert)
{
    var xhr = new XMLHttpRequest()
    xhr.open(method,url,true)
    xhr.onload = function() {
        var response;
        try {
            response = JSON.parse(xhr.responseText);
        }
        catch(e) { 
            on_error(e); 
            return;
        }
        if('status' in response && response.status == 'fail') {
            on_error(response.error);
            return;
        }
        on_response(response)
    };
    if(request != null) {
        xhr.setRequestHeader('Content-Type','application/json');
        xhr.send(JSON.stringify(request));
    }
    else {
        xhr.send();
    }
}

function getStoageValue()
{
    var storage = window.localStorage;
    var data = storage.getItem('ols_data');
    if(!data)
        return {};
    console.log('Loaded:' + data);
    return JSON.parse(data);
}

function setStorageValue(obj)
{
    var data = JSON.stringify(obj);
    console.log('Saved:' + data);
    window.localStorage.setItem('ols_data',data);
}

function loadSavedInputValue(id)
{
    var storage = getStoageValue();
    if(id in storage)
        return storage[id];
    return null;
}

function saveInputValue(id)
{
    var val = document.getElementById(id);
    if(!val)
        return;
    var st = getStoageValue();
    st[id] = val.value;
    setStorageValue(st);
}

function updateSavedInputs()
{
    var els = document.getElementsByClassName('saved_input');
    for(var i=0;i<els.length;i++) {
        var el = els[i];
        var id = el.id;
        console.log("Loading " + id);
        var val = loadSavedInputValue(id);
        if(val!=null) {
            el.value = val;
            el.dispatchEvent(new Event("input"));
        }
        el.addEventListener('input',(e)=> { saveInputValue(e.currentTarget.id); });
    }
}

function run()
{
    restCall("get","/api/camera",null,listCameras);
    if(navigator.geolocation) {
        navigator.geolocation.getCurrentPosition((pos)=>{
            document.getElementById("stack_lat").value=pos.coords.latitude.toFixed(2);
            document.getElementById("stack_lon").value=pos.coords.longitude.toFixed(2);
        });
    }
    updateSavedInputs();
}


function listCameras(cameras)
{
    var sel = document.getElementById('camera_select')
    for(var i=0;i<cameras.length;i++) {
        var option = document.createElement("option");
        option.text = cameras[i].name
        option.value = cameras[i].id.toString()
        sel.add(option)
    }
    restCall("get","/api/camera/status",null,checkOpenStatus);
}

function updateStackSelect()
{
    var type = getVal('type');
    var elements = document.getElementsByClassName('dso_config');
    var new_style = type == 'dso' ? '' : 'none';
    for(var i=0;i<elements.length;i++) {
        elements[i].style.display = new_style;
    }
}

function checkOpenStatus(st)
{
    if(st.status == 'closed') {
        return;
    }
    else if(st.status == 'open') {
        loadFormats();
        loadControls();
    }
    else if(st.status == 'streaming') {
        global_width = st.width;
        global_height = st.height;
        loadFormats();
        loadControls();
        showConfig(false);
        showLiveVideo();
        restCall('get','/api/stacker/status',null, (d)=> {
            changeStackerStatus(d.status);
        });
        recalcFOV();
    }
}

function changeStackerStatus(new_status)
{
    var ui_ctl = {
        stack: true,
        pause: false,
        cancel: false,
        resume: false,
        solve: true,
        save: false,
        live: false,
        pp: false,
        stats: false
    };
    if(new_status == 'idle') {
        if(g_stacker_status != 'idle') {
            showLiveVideo();
            showThumbLive(false);
        }
        ui_ctl.stack = true;
        ui_ctl.pause = false;
        ui_ctl.resume = false;
        ui_ctl.save=false;
        ui_ctl.cancel=false;
        ui_ctl.pp=false;
        ui_ctl.live = false;
        ui_ctl.stats = false;
    }
    else if(new_status =='stacking') {
        if(g_stacker_status == 'idle') {
            showStackedVideo(true);
        }
        ui_ctl.stack = false;
        ui_ctl.cancel = true;
        ui_ctl.pause = true;
        ui_ctl.resume = false;
        ui_ctl.save = true;
        ui_ctl.pp = true;
        ui_ctl.live = true;
        ui_ctl.stats = true;
    }
    else if(new_status == 'paused') {
        if(g_stacker_status == 'idle') {
            showStackedVideo(true);
        }
        ui_ctl.stack = false;
        ui_ctl.pause = false;
        ui_ctl.cancel = true;
        ui_ctl.resume = true;
        ui_ctl.save = true;
        ui_ctl.pp = true;
        ui_ctl.live = true;
        ui_ctl.stats = true;
    }
    else {
        alert('Internal error invalud status' + new_status);
    }
    for(var name in ui_ctl) {
        var st = ui_ctl[name] ? 'inline' : 'none';
        document.getElementById('stacking_ui_' + name).style.display = st;
    }
    if(ui_ctl.stats) {
        if(!g_stats) {
            g_stats = new EventSource("/api/updates");
            g_stats.onmessage = (e) => {
                updateStackerStats(e.data);
            };
        }
    }
    else {
        if(g_stats) {
            g_stats.close();
            g_stats = null;
        }
    }
    g_stacker_status = new_status;
}

function updateStackerStats(e) {
    var stats = JSON.parse(e);
    document.getElementById('stats_info').innerHTML = stats.stacked + '/' + stats.missed + '/' + stats.dropped;
}

function loadFormats()
{
    document.getElementById('open_camera').style.display='none';
    document.getElementById('std_controls').style.display='inline';
    showConfig(true);
    restCall('get','/api/camera/formats', null, (formats)=> {
        var sel = document.getElementById('stream_format')
        for(var i=sel.length-1;i>=0;i--) {
            sel.options.remove(i);
        }
        for(var i=0;i<formats.length;i++) {
            var option = document.createElement("option");
            option.text = formats[i].format_id
            option.value = formats[i].format_id
            sel.add(option)
        }
    });
}

function openCamera()
{
    var cam_id = document.getElementById('camera_select').value;
    restCall('post','/api/camera', { operation: 'open',id:parseInt(cam_id) }, function(msg) {
        loadFormats();
        loadControls();
    });
}

function updateDependent(opt,enable)
{
    enableNumControl(opt, enable);
    var obj = document.getElementById('control_opt_' + opt);
    if(obj) {
        obj.readOnly = !enable;
        if(enable) {
            reloadNumControl(opt);
        }
    }
}

function updateAllDependent(optid,value)
{
    var enable = value ? false : true;
    if(optid == 'auto_exp') {
        updateDependent('exp',enable);
        updateDependent('gain',enable);
    }
    else if(optid == 'auto_wb') {
        updateDependent('wb',enable);
        updateDependent('wb_r',enable);
        updateDependent('wb_b',enable);
    }
}

function updateControl(optid,value)
{
    restCall('post','/api/camera/option/' + optid,{'value':value},(e)=>{
        enableNumControl(optid,false);
        updateAllDependent(optid,value);
    });
}
function updateBoolControl(optid,value)
{
    updateControl(optid,value ? 1 : 0);
}

function enableNumControl(optid,val)
{
    var ctl = document.getElementById('control_button_' + optid);
    if(ctl) {
        ctl.disabled = !val;
    }
}

function updateNumControl(optid)
{
    var val = document.getElementById('control_opt_' + optid).value;
    val = parseFloat(val);
    updateControl(optid,val);
    enableNumControl(optid,false);
    return false;
}


function reloadNumControl(optid)
{
    if(!document.getElementById("control_opt_" + optid))
        return;
    restCall('get','/api/camera/option/' + optid,null,(e)=>{
        document.getElementById("control_opt_" + optid).value = e.value + '';
    });
}

function prepareControls(ctls)
{
    var controls = document.getElementById('cam_controls');
    var controls_str = '';
    var has_aexp = false, has_awb = false;
    var aexp_on = false, awb_on = false;
    for(var i=0;i<ctls.length;i++) {
        var ctl = ctls[i];
        var disabled = false;
        if(ctl.option_id == 'auto_wb') {
            has_awb = true;
            awb_on = ctl.cur != 0;
            console.log('AWB on=' + awb_on);
        }
        else if(ctl.option_id == 'auto_exp') {
            has_aexp = true;
            aexp_on = ctl.cur != 0;
            console.log('AEXP on=' + aexp_on);
        }
        else if(ctl.option_id == 'wb' || ctl.option_id == 'wb_r' || ctl.option_id == 'wb_b') {
            if(has_awb && awb_on)
                disabled = true;
        }
        else if(ctl.option_id == 'exp' || ctl.option_id == 'gain') {
            if(has_aexp && aexp_on)
                disabled = true;
        }
        if(disabled)
            console.log('Option ' + ctl.option_id + ' started disbaled')
        else
            console.log('Option ' + ctl.option_id + ' started enabled')
        var unit='';
        if(ctl.type == 'percent')
            unit='(%)';
        else if(ctl.type == 'msec')
            unit='(ms)';
        else if(ctl.type == 'kelvin')
            unit='(K)';
        else if(ctl.type == 'celsius')
            unit='(C)';
        var control_str = `<td>${ctl.name}${unit}</td>`;
        if(ctl.type == 'bool') {
            var checked = ctl.cur != 0 ? 'checked' : '';
            control_str += `<td><input type="checkbox" ${checked} onchange="updateBoolControl('${ctl.option_id}',this.checked ? 1 : 0);"/></td>`;
            control_str += '<td>&nbsp;</td>';
            control_str += '<td>&nbsp;</td>';
            control_str += '<td>&nbsp;</td>';
        }
        else if(ctl.read_only) {
            control_str +=`<td><input class="val_input" id="control_opt_${ctl.option_id}" type="text" value="${ctl.cur}" readonly</td>`;
            control_str += `<td><button id="control_button_${ctl.option_id}" onclick="reloadNumControl('${ctl.option_id}');" >get</button></td>`;
            control_str += '<td>&nbsp;</td>';
            control_str += '<td>&nbsp;</td>';
        }
        else {
            var injected = disabled ? 'readonly' : '';
            control_str +=`<td><form onsubmit="return updateNumControl('${ctl.option_id}');" ><input class="val_input" id="control_opt_${ctl.option_id}" min="${ctl.min}" max="${ctl.max}" step="${ctl.step}" type="number" value="${ctl.cur}" oninput="enableNumControl('${ctl.option_id}',true);"  ${injected}  /></form></td>`;
            control_str += `<td><button disabled id="control_button_${ctl.option_id}" onclick="updateNumControl('${ctl.option_id}');" >set</button></td>`;
            var ctl_max = ctl.max;
            if(ctl_max > 1000 && ctl_max % 1000 == 0 && ctl.type == "msec") {
                ctl_max = (ctl_max / 1000) + 's';
            }
            control_str +=`<td>${ctl.min}&ndash;${ctl_max}</td>`;
            control_str +=`<td>${ctl.default}</td>`;
        }
        controls_str += '<tr>' + control_str + '</tr>\n';
    }
    controls.innerHTML = controls_str;
}

function loadControls()
{
    restCall('get','/api/camera/options',null,prepareControls);
}

function calcImgSize()
{
    var W = window.innerWidth;
    var H = window.innerHeight;
    var img_H,img_W;
    var p_top=0,p_left=0;
    var w2h = global_width/global_height
    if(W/H > w2h) {
        img_H = H;
        img_W = H*w2h;
        p_left = (W - img_W)/2;
    }
    else {
        img_W = W;
        img_H = W/w2h;
        p_top = (H - img_H) / 2;
    }
    return [img_W,img_H,p_left,p_top]
}

function updateCalibFramesById(data,elemid)
{
    var sel = document.getElementById(elemid);
    var selected = sel.value;
    for(var i=sel.length-1;i>=0;i--) {
        sel.options.remove(i);
    }
    var found=false;
    for(var i=-1;i<data.length;i++) {
        var fid = 'N/A';
        var width=global_width;
        var height=global_height;
        if(i >= 0) {
            fid = data[i].id;
            width = data[i].width;
            height = data[i].height;
        }
        if(width != global_width || height != global_height)
            continue;
        var option = document.createElement("option");
        option.text = fid;
        option.value = fid;
        sel.add(option)
        if(fid == selected)
            found=true;
    }
    sel.value = selected;
}

function updateCalibFrames(data)
{
    updateCalibFramesById(data,'stack_darks');
    updateCalibFramesById(data,'stack_flats');
    updateCalibFramesById(data,'stack_dark_flats');
}

function loadCalibFrames()
{
    restCall('get','/data/calibration/index.json',null,updateCalibFrames);
}

function showStackedVideo(stacked_on)
{
    if(stacked_on) {
        showStream('stacked')
    }
    else
        showStream('live')
}

function showStream(name)
{
    document.getElementById('video').innerHTML = `<img id="live_stream_video" alt="streaming video" src="/api/video/${name}" />`;
    onResize(null);
}

function toggleThumbLive()
{
    showThumbLive(!g_show_thumb_live);
}

function showThumbLive(v)
{
    var thumb = document.getElementById('thumb');
    var thumb_video = document.getElementById('thumb_video');
    g_show_thumb_live = v;
    var live_button = document.getElementById('stacking_ui_live');
    if(v) {
        thumb_video.innerHTML = `<img id="thumb_stream_video" alt="streaming video" src="/api/video/live" />`;
        thumb.style.display = 'inline';
        live_button.innerHTML = 'X';
        onResize(null);
    }
    else {
        thumb_video.innerHTML ='';
        thumb.style.display = 'none';
        live_button.innerHTML = 'L';
    }
}

function showLiveVideo()
{
    showStream('live');
}

function onResize(ev)
{
    var video = document.getElementById('live_stream_video');
    var size = calcImgSize();
    if(video) {
        video.style.width = size[0] + 'px';
        video.style.height = size[1] + 'px';
        video.style.marginLeft = size[2] + 'px';
        video.style.marginTop  = size[3] + 'px';
    }
    var thumb = document.getElementById('thumb_stream_video')
    if(thumb) {
        thumb.style.width = Math.round(size[0] / 4) + 'px';
        thumb.style.height = Math.round(size[1] / 4) + 'px';
        thumb.style.marginRight = '0px';
        thumb.style.marginTop  = '0px';
    }
    var solved = document.getElementById('solver_result_image');
    if(solved) {
        solved.style.width = Math.round(size[0] / 2) + 'px';
        solved.style.height = Math.round(size[1] / 2) + 'px';
    }
}

function startStream()
{
    var format_id = document.getElementById('stream_format').value;
    var max_fr = parseFloat(document.getElementById('stream_max_framerate').value);
    if(isNaN(max_fr))
        max_fr = 0;
    restCall('post','/api/camera/stream',{op:'start',format_id:format_id, max_framerate: max_fr },(r)=>{
        global_width = r.width;
        global_height = r.height;
        showLiveVideo();
        changeStackerStatus('idle');
        recalcFOV();
    });
    showConfig(false);
}

function showStack(v)
{
    if(v) {
        loadCalibFrames();
    }
	document.getElementById('stack').style.display = v ? 'inline' : 'none';
}

function showPP(v)
{
	document.getElementById('pp_control').style.display = v ? 'inline' : 'none';
}

function showConfig(v)
{
	document.getElementById('config').style.display = v ? 'inline' : 'none';
}

function getVal(name)
{
    return document.getElementById('stack_' + name).value;
}

function getBVal(name)
{
    return document.getElementById('stack_' + name).checked;
}

function getPVal(name)
{
    return document.getElementById('stack_' + name).value * 0.001;
}

function ctlStack(status)
{
    restCall('post','/api/stacker/control',{operation:status},(s)=> {
        if(status == 'save' || status == 'cancel') {
            changeStackerStatus('idle');
        }
        else if(status == 'resume') {
            changeStackerStatus('stacking');
        }
        else if(status =='pause') {
            changeStackerStatus('paused');
        }
    });
}

function setPPVal(name,val)
{
    document.getElementById('stack_stretch_' + name + '_disp').innerHTML = val.toFixed(3);
}



function setPPSlider(name,val)
{
    document.getElementById('stack_stretch_' + name).value = Math.round(val*1000);
    setPPVal(name,val);
}

function updatePPSliders(data)
{
    setPPSlider('low',data.cut);
    setPPSlider('high',data.gain);
    setPPSlider('gamma',data.gamma);
    g_prev_low = data.cut;
    g_prev_high = data.gain;
    updatePP();
}

function setDefaultPPSliders(e=null)
{
    updatePPSliders({
        gamma: 2.4,
        cut : 0.0,
        gain : 1.0
    });
}

function updateAutoPP()
{
    var auto_pp = getBVal("auto_stretch");
    if(auto_pp) {
        document.getElementById('dynamic_parameters').style.display='none';
        updatePP();
        return;
    }
    document.getElementById('dynamic_parameters').style.display='inline';
    restCall('get','/data/stretch.json',null,updatePPSliders,setDefaultPPSliders);
}

function updatePPLow()
{
    g_prev_low = getPVal('stretch_low');
    g_prev_high = getPVal("stretch_high");
    updatePP();
}

function updatePPHigh()
{
    var new_high = getPVal("stretch_high");
    if(g_prev_low != -1 && g_prev_high != -1) {
        var new_low = Math.min(Math.max(g_prev_low,0.8),g_prev_low * (new_high / g_prev_high));
        setPPSlider('low',new_low);
    }
    else {
        g_prev_high = new_high;
        g_prev_low = getPVal("stretch_low");
    }
    updatePP();
}

function updatePP()
{

    var config={
        auto_stretch:       getBVal("auto_stretch"),
        stretch_low:        getPVal('stretch_low'),
        stretch_high:       getPVal('stretch_high'),
        stretch_gamma:      getPVal('stretch_gamma'),
    };
    setPPVal('low',config.stretch_low);
    setPPVal('high',config.stretch_high);
    setPPVal('gamma',config.stretch_gamma);
    restCall('post','/api/stacker/stretch',config,(e)=>{
    });
}

function startStack()
{
    var darks = getVal('darks');
    if(darks == 'N/A') {
        darks = null;
    }
    var flats = getVal('flats');
    if(flats == 'N/A') {
        flats = null;
    }
    var dark_flats = getVal('dark_flats');
    if(dark_flats == 'N/A') {
        dark_flats = null;
    }
    var ra=parseRA(getVal("ra"));
    var de=parseDEC(getVal("de"));
    var lat = parseFloat(getVal("lat"));
    var lon = parseFloat(getVal("lon"));
    var field_derotation = getBVal("field_derotation");
    if(field_derotation) {
        if(isNaN(lat) || isNaN(lon)) {
            alert('Need Geolocation for derotation support\n(see settings menu)');
            return;
        }
        if(ra==null || de==null) {
            alert('Need RA, DE for derotation');
            return;
        }
    }
    if(isNaN(lat))
        lat = 0;
    if(isNaN(lon))
        lon = 0;

    var name = getVal('name').trim();
    var obj = getVal('object').trim();

    var type = getVal('type');
    var calib = type != 'dso';
    if(!calib) {
        if(name == '')
            name = obj;
        else if(obj != '')
            name = name + '_' + obj;
    }
    name = name.replace(/ /g,'_');
    const name_pat = /^[A-Za-z0-9_.\-]*$/g;
    if(!name.match(name_pat)) {
        alert('Name and Object should contain only English letters, digits, "_", "-" and "."')
        return;
    }
    if(calib && name=='') {
        alert('Provide name for calibration frame');
        return;
    }

    var config={
        name:               name,
        save_data:          getBVal("save_data"),
        field_derotation:   field_derotation,
        image_flip:         getBVal("image_flip"),
        remove_satellites:  getBVal("remove_satellites"),
        auto_stretch:       getBVal("auto_stretch"),
        stretch_low:        getPVal('stretch_low'),
        stretch_high:       getPVal('stretch_high'),
        stretch_gamma:      getPVal('stretch_gamma'),
        type:               type,
        location : {
            lat:            lat,
            lon:            lon,
        },
        target: {
            ra:             ra,
            de:             de,
        },
        darks : darks,
        flats : flats,
        dark_flats : dark_flats
    };
    restCall('post','/api/stacker/start',config,(e)=>{
        changeStackerStatus('stacking');
        showStack(false);
    });
}

function updateRADE(name)
{
    updateRADEFor(name,'stack');
}
function updateSolverRADE(name)
{
    updateRADEFor(name,'solver');
}

function updateRADEFor(name,target_id)
{
    var coord=['','']
    name = name.toUpperCase();
    if(name in jsdb) {
        coord = jsdb[name]
    }
    document.getElementById(target_id + '_ra').value = coord[0];
    document.getElementById(target_id + '_de').value = coord[1];
}

function recalcFOV()
{
    try {
        var pixel_size = parseFloat(document.getElementById('solver_pixel_size').value);
        var FL = parseFloat(document.getElementById('solver_focal_length').value);
        if(isNaN(pixel_size) || isNaN(FL))
            return;
        var fov = global_height * pixel_size * 1e-3 / FL * 180 / 3.14159;
        fov = fov.toFixed(3);
        console.log('Setting FOV ' + fov);
        document.getElementById('solver_fov').value = fov;
    }
    catch(e) {
        console.log('calcing FOV failed' + e)
    }

}

function showSolve(show)
{
    document.getElementById('plate_solve_div').style.display = show ? 'inline' : 'none';
}


function toggleFS(fs)
{
    if(fs) {
        document.documentElement.requestFullscreen({navigationUI:'hide'});
    }
    else {
        document.exitFullscreen();
    }
}


function plateSolve()
{
    var ra,de,fov,rad,lat,non;
    try {
        ra = parseRA(document.getElementById('solver_ra').value);
        de = parseDEC(document.getElementById('solver_de').value);
        fov = parseFloat(document.getElementById('solver_fov').value);
        rad = parseFloat(document.getElementById('solver_radius').value);
        lat = parseFloat(getVal("lat"));
        lon = parseFloat(getVal("lon"));
        if(isNaN(lat))
            lat = null;
        if(isNaN(lon))
            lon = null;
    }
    catch(e) {
        alert('Invalid Inputs' + e)
        return;
    }
    document.getElementById('solver_config').style.display = 'none';
    document.getElementById('solver_status_div').style.display = 'inline';
    document.getElementById('solver_status').innerHTML = 'Waiting...';
    document.getElementById('solver_result').style.display = 'none';
    var req = {
        "fov" : fov,
        "ra"  : ra,
        "de"  : de,
        "rad" : rad,
        "lat" : lat,
        "lon" : lon
    };
    restCall('post','/api/plate_solver',req,solveResult)
}

function formatAngle(deg,as_hours)
{
    if(as_hours) {
        deg = deg / 15;
        sep = "h";
    }
    else {
        sep = "\u00b0";
    }
	var direction = deg >= 0 ? '+' : '-';
	deg = Math.abs(deg);
	var ideg = Math.floor(deg);
	var min = 60*(deg - ideg);
	var imin = Math.floor(min);
	var sec = 60*(min - imin);
    var msg = direction + ideg + sep + imin + "'";
    if(as_hours)
	    msg += Math.floor(sec) + "''";
    return msg;
}

function formatTableRow(a)
{
    var res = '<tr>';
    for(var i=0;i<a.length;i++) {
        res += '<td>' + a[i] + '</td>';
    }
    res += '</tr>\n';
    return res;
}

function solveResult(v)
{
    var status, jpeg = null;
    if(v.solved) {
        jpeg = v.result_image + '?' + Date.now();
        status = 'Distance to target ' + v.distance_to_target.toFixed(2) + '°';
        var delta_ra_hms = formatAngle(v.delta_ra,true);
        var delta_ra_dms = formatAngle(v.delta_ra,false);
        var delta_ra_deg = v.delta_ra.toFixed(2) + "\u00b0";
        var delta_de_dms = formatAngle(v.delta_de,false);
        var delta_de_deg = v.delta_de.toFixed(2) + "\u00b0";
        status += '<table>'
        status += formatTableRow(['∆RA',delta_ra_dms,delta_ra_deg,delta_ra_hms,'∆DEC',delta_ra_dms,delta_ra_deg]);
        if(v.delta_az) {
            var delta_az_dms = formatAngle(v.delta_az,false);
            var delta_az_deg = v.delta_az.toFixed(2) + "\u00b0";
            var delta_alt_dms = formatAngle(v.delta_alt,false);
            var delta_alt_deg = v.delta_alt.toFixed(2) + "\u00b0";
            status += formatTableRow(['∆ALT',delta_alt_dms,delta_alt_deg,'&nbsp;','∆AZ',delta_az_dms,delta_az_deg]);
        }
        status += '</table>'
        if(!v.delta_az) {
            status += 'for ∆Alt/∆AZ provide geolocation in settings';
        }
    }
    else {
        status = v.error;
    }
    document.getElementById('solver_status').innerHTML = status;
    if(jpeg) {
        document.getElementById('solver_result_image').src = jpeg;
    }
    document.getElementById('solver_result').style.display = jpeg ? 'inline' : 'none';
}

function solverRestart()
{
    document.getElementById('solver_status_div').style.display='none';
    document.getElementById('solver_config').style.display='inline';
}


window.onload = (event) => {
  run();
};

window.onresize = onResize;


