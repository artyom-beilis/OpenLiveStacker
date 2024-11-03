var g_slew_max = -1;
var g_slew_speed = 5;
var global_width = 0;
var global_height = 0;
var global_bin = 0;
var global_download_progress = null;
var global_zoom = 1.0;
var g_sharpen_sent = 0.0;
var g_sharpen_delay = false;
var g_stacker_status = 'closed';
var g_show_thumb_live = false;
var g_stats = null;
var g_solving = false;
var g_since_saved_s = 0;
var g_solving_ui_open = false;
var g_error_messages = {};
var g_profile_cam_opts = null;
var g_solver_result = null;
var g_confirm_callback = null;
var g_histogram = null;
var g_show_hist = false;
var g_hist_zoom = 'vis';
var g_hist_move_line_id = -1;
var g_hist_move_line_pos = -1;
var g_hist_move_start_x = -1;
var g_mount_connected = false;
var g_mount_driver_populated = null;
var g_sync_counter = 0;
var g_mount_geolocation_updated = false;
var g_stretch = {
    gain:1,
    cut:0,
    gamma:2.2
};

function zoom(offset)
{
    if(offset > 0 && global_zoom < 4)
        global_zoom += offset;
    else if(offset < 0 && global_zoom > 1)
        global_zoom += offset;
    var zspan = document.getElementById('current_zoom')
    if(global_zoom == 1)
        zspan.innerHTML = '';
    else
        zspan.innerHTML = '×' + global_zoom;
    onResize(null);
}

function requestConfirmation(message,callback)
{
    showAlertDialog(message,callback);
}

function showError(message)
{
    showAlertDialog(message,null);
}

function showAlertDialog(message,confirm_callback)
{
    document.getElementById('error_message').innerHTML = message;
    document.getElementById('error_alert').style.display = 'inline';
    var cancel = document.getElementById('confirm_cancel_button');
    g_confirm_callback = confirm_callback;
    if(confirm_callback!=null) {
        cancel.style.display='inline';
    }
    else {
        cancel.style.display='none';
    }
}
function confirmError()
{
    var callback = g_confirm_callback;
    hideError();
    if(callback!=null)
        callback();
}

function hideError()
{
    g_confirm_callback = null;
    document.getElementById('error_alert').style.display = 'none';
}

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
    const re_float = /^([+-−]?\d+\.\d+?)$/;
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
        s = parseFloat(res[4]);
    }
    if(d > 90 || m >= 60 || s >= 60)
        return null;
    return sig*(d + (60 * m + s) / 3600.0);
}

function restCall(method,url,request,on_response,on_error = showError)
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
    return JSON.parse(data);
}

function setStorageValue(obj)
{
    var data = JSON.stringify(obj);
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
    var value = null;
    if(val.tagName == 'INPUT') {
        if(val.type=='checkbox')
            value = val.checked;
        else
            value = val.value;
    }
    else if(val.tagName == 'SELECT') {
        value = val.value;
    }
    if(value != null) {
        st[id] = value;
        setStorageValue(st);
    }
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
            if(el.tagName == 'INPUT') {
                if(el.type == 'checkbox')
                    el.checked = val;
                else
                    el.value = val;
            }
            else if (el.tagName == 'SELECT') {
                el.value = val;
            }
            el.dispatchEvent(new Event("input"));
            el.dispatchEvent(new Event("change"));
        }
        el.addEventListener('input',(e)=> { saveInputValue(e.currentTarget.id); });
    }
}

function setGPS(lat,lon)
{
    document.getElementById("stack_lat").value=lat.toFixed(2);
    document.getElementById("stack_lon").value=lon.toFixed(2);
}

function getQueryParameters()
{
    var url=window.location.href;
    var queryPos = url.indexOf('?');
    var vals={};
    if(queryPos == -1)
        return vals;
    var params = url.slice(queryPos + 1).split('&');
    for(var i=0;i<params.length;i++) {
        var param = params[i];
        var data = param.split('=');
        if(data.length == 2) {
            vals[data[0]] = parseFloat(data[1]);
        }
    }
    return vals;
}

function checkParams(params)
{
    if('android_view' in params && params.android_view == 1) {
        document.getElementById('FS_tooggle').style.display = 'none'
    }
}

function checkAndSetLocation(params)
{
    if('lat' in params && 'lon' in params) {
        setGPS(params.lat,params.lon);
        return true;
    }
    return false;
}

function setPressUnpressEvents(id,on_event,off_event)
{
    var but = document.getElementById(id);
    but.addEventListener('mousedown',(e)=>{on_event();});
    but.addEventListener('touchstart',(e)=>{on_event();});

    but.addEventListener('mouseup',(e)=>{off_event();});
    but.addEventListener('mouseout',(e)=>{off_event();});
    but.addEventListener('touchend',(e)=>{off_event();});
    but.addEventListener('touchcancel',(e)=>{off_event();});
}

function setSlewEventListeners()
{
    var dirs = ['N','S','W','E'];
    for(var i=0;i<dirs.length;i++) {
        let d = dirs[i];
        setPressUnpressEvents(`slew_${d}`,
            ()=>{
                console.log('Slew  to ' + d);
                slew(d);
            },
            noslew);
    }
    setPressUnpressEvents('mount_manual_sync',manualSyncPressStart,manualSyncPressEnd);
}

function setEventListeners()
{
    setCanvasEventListeners();
    setSlewEventListeners();
}

function setCanvasEventListeners()
{
    var canvas = document.getElementById('hist_canvas');
    canvas.addEventListener('mousedown',(e)=>histEvent(e,'down'))
    canvas.addEventListener('mouseup',(e)=>histEvent(e,'up'))
    canvas.addEventListener('mouseout',(e)=>histEvent(e,'up'))
    canvas.addEventListener('mousemove',(e)=>histEvent(e,'move'))

    canvas.addEventListener('touchstart',(e)=>histEvent(e.touches[0],'down'))
    canvas.addEventListener('touchend',(e)=>histEvent(null,'end'))
    canvas.addEventListener('touchcancel',(e)=>histEvent(null,'end'))
    canvas.addEventListener('touchmove',(e)=>histEvent(e.touches[0],'move'))
}


function run()
{
    var params = getQueryParameters();
    console.log(params)
    checkParams(params);
    restCall("get","/api/camera",null,listCameras);
    setEventListeners();
    updateSavedInputs();
    update_mnt();
    startUIUpdates();
    if(!checkAndSetLocation(params) && navigator.geolocation) {
        navigator.geolocation.getCurrentPosition((pos)=>{
            setGPS(pos.coords.latitude,pos.coords.longitude);
        });
    }
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
    var elements = document.getElementsByClassName('stack_opt');
    for(var i=0;i<elements.length;i++) {
        elements[i].style.display = 'none';
    }
    if(type == 'dso')
        elements = document.getElementsByClassName('dso_config');
    else if(type == 'calibration')
        elements = document.getElementsByClassName('calib_config');
    else if(type == 'planetary')
        elements = document.getElementsByClassName('plan_config');
    else if(type == 'dynamic')
        elements = document.getElementsByClassName('dynamic_config');
    new_style = '';
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
        global_bin = st.bin;
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
        stop:true,
        pause: false,
        cancel: false,
        resume: false,
        solve: true,
        save: false,
        live: false,
        pp: false,
        sharp: false,
        stats: false
    };
    var type = getVal('type');
    if(new_status == 'closed') {
        ui_ctl.stack = false;
        ui_ctl.solve = false;
        ui_ctl.stop = false;
        ui_ctl.pause = false;
        ui_ctl.resume = false;
        ui_ctl.save=false;
        ui_ctl.cancel=false;
        ui_ctl.pp=false;
        ui_ctl.shapr=false;
        ui_ctl.live = false;
        ui_ctl.stats = false;
    }
    else if(new_status == 'idle') {
        if(g_stacker_status != 'idle') {
            showLiveVideo();
            showThumbLive(false);
        }
        ui_ctl.stack = true;
        ui_ctl.solve = true;
        ui_ctl.stop = true;
        ui_ctl.pause = false;
        ui_ctl.resume = false;
        ui_ctl.save=false;
        ui_ctl.cancel=false;
        ui_ctl.pp=false;
        ui_ctl.sharp=false;
        ui_ctl.live = false;
        ui_ctl.stats = false;
    }
    else if(new_status =='stacking') {
        if(g_stacker_status == 'idle' || g_stacker_status == 'closed') {
            showStackedVideo(true);
        }
        ui_ctl.stack = false;
        ui_ctl.solve = true;
        ui_ctl.stop = false;
        ui_ctl.cancel = true;
        ui_ctl.pause = true;
        ui_ctl.resume = false;
        ui_ctl.save = true;
        ui_ctl.pp = type == 'dso' || type == 'dynamic';
        ui_ctl.sharp = type == 'planetary';
        ui_ctl.live = true;
        ui_ctl.stats = true;
    }
    else if(new_status == 'paused') {
        if(g_stacker_status == 'idle' || g_stacker_status == 'closed') {
            showStackedVideo(true);
        }
        ui_ctl.stack = false;
        ui_ctl.stop = false;
        ui_ctl.pause = false;
        ui_ctl.cancel = true;
        ui_ctl.resume = true;
        ui_ctl.save = true;
        ui_ctl.pp = type == 'dso' || type == 'dynamic';
        ui_ctl.sharp = type == 'planetary';
        ui_ctl.live = true;
        ui_ctl.stats = true;
    }
    else {
        showError('Internal error invalud status' + new_status);
    }
    for(var name in ui_ctl) {
        var st = ui_ctl[name] ? 'inline' : 'none';
        document.getElementById('stacking_ui_' + name).style.display = st;
    }
    g_stacker_status = new_status;
}

function startUIUpdates()
{
    if(!g_stats) {
        g_stats = new EventSource("/api/updates");
        g_stats.onmessage = (e) => {
            updatePushEvents(e.data);
        };
    }
}

function hZoom(value)
{
    g_hist_zoom = value;
    updateHistogram();
}

function histEvent(e,type)
{
    var canvas = document.getElementById('hist_canvas');
    var rect = canvas.getBoundingClientRect();
    var x = null;
    if(e!=null) {
        x = e.clientX - rect.left;
    }
    var tr = getHistTrans();
    var pos = [ tr.x_black, tr.x_med, tr.x_white ];
    if(type == 'down') {
        var index = -1;
        var maxDiff = -1;
        for(let i=0;i<3;i++) {
            let lineX = tr.getX(pos[i]);
            let diff = Math.abs(lineX - x);
            if(index == -1 || maxDiff > diff) {
                index = i;
                maxDiff = diff;
            }
        }
        if(maxDiff > 0.1 * canvas.width) {
            g_hist_move_line_id = -1;
        }
        else {
            g_hist_move_line_id = index;
            g_hist_move_line_pos = pos[index];
            g_hist_move_start_x = x;
        }
    }
    else if(type == 'move' && g_hist_move_line_id != -1) {
        let delta = (x - g_hist_move_start_x) / tr.h2p_a;
        g_hist_move_line_pos = pos[g_hist_move_line_id] + delta;
    }
    else if(type == 'up' && g_hist_move_line_id != -1) {
        let delta = (x - g_hist_move_start_x) / tr.h2p_a;
        pos[g_hist_move_line_id] += delta;
        recalcStretch(pos[0],pos[1],pos[2],g_hist_move_line_id);
        g_hist_move_line_id = -1;
    }
    else if(type == 'end' && g_hist_move_line_id != -1) {
        pos[g_hist_move_line_id] = g_hist_move_line_pos;
        recalcStretch(pos[0],pos[1],pos[2],g_hist_move_line_id);
        g_hist_move_line_id = -1;
    }
    updateHistogram();
}

function recalcStretch(black,med,white,index)
{
    if(black < 0)
        black = 0;
    if(white > 1)
        white = 1;
    if(black >= white)
        return;
    var gain = 1.0/(white - black);
    if(gain > 32)
        gain = 32;
    var cut = black * gain; 
    var gamma = g_stretch.gamma;
    if(index == 1) { // change gamma IFF it is requested
        let medp = (med - black) / (white - black);
        let medp_limit = Math.pow(0.5,8); // gamma=8 max
        if(medp > 0.5)
            medp = 0.5;
        if(medp < medp_limit)
            medp = medp_limit;
        gamma = Math.log(medp) / Math.log(0.5);
    }
    updatePPSliders({
        gain:gain,
        gamma: gamma,
        cut:cut
    });
}

function getHistTrans()
{
    var canvas = document.getElementById('hist_canvas');
    var w = canvas.width;
    var h = canvas.height;
    var sp = getStretchParams();
    var vis_gamma_x = Math.pow(0.5,sp.gamma);
    // x_v = x_h * sp.gain - sp.cut
    // x_h = (x_v + sp.cut) / sp.gain
    var x0,x1;
    var x_black = sp.cut / sp.gain;
    var x_white = (1 + sp.cut)/sp.gain;
    var x_med = x_black * (1.0 - vis_gamma_x) + x_white * vis_gamma_x;
    if(g_hist_zoom == 'all') {
        x0 = 0;
        x1 = 1;
    }
    else if(g_hist_zoom == 'vis') {
        x0 = x_black;
        x1 = x_white;
    }
    else if(g_hist_zoom == '0' || g_hist_zoom == '1' || g_hist_zoom == '0.5') {
        var scale = 0.1;
        var vis_center = 0.5;
        if(g_hist_zoom == '0')
            vis_center = 0;
        else if(g_hist_zoom == '1')
            vis_center = 1;
        else if(g_hist_zoom == '0.5')
            vis_center = vis_gamma_x;
        x0 = (vis_center - scale * 0.5 + sp.cut) / sp.gain;
        x1 = (vis_center + scale * 0.5 + sp.cut) / sp.gain;
    }
    /// h2p = (x_h) => (x_h - x0)/(x1-x0) * w*0.5 + w*0.25
    return {
        h2p_a : 1.0/(x1-x0) * w*0.5,
        h2p_b : -x0/(x1-x0) * w*0.5 + w*0.25,
        getX : function(x)  { return this.h2p_a * x + this.h2p_b; },
        getY : function(y)  { return this.y2p_a * y + this.y2p_b; },
        getXfromPix : function(x) { return (x - this.h2p_b) / this.h2p_a; },
        y2p_a : -0.8*h,
        y2p_b : 0.9*h,
        x_black: x_black,
        x_white : x_white,
        x_med : x_med
    };
}

function getStretchParams()
{
    return g_stretch;
}

function gammaToAsinh(pw)
{
    if(pw >= 1)
        return 1.0;
    var half_point = Math.pow(0.5,1.0/pw);
    var a = 1.0/half_point * Math.sinh(Math.acosh(1.0/(2.0*half_point)));
    return a;
}

function updateHistogram()
{
    var div = document.getElementById('hist_div')
    var bdiv = document.getElementById('hist_ctl')
    if(!g_show_hist) {
        div.style.display = 'none';
        bdiv.style.display = 'none';
        return;
    }
    else {
        div.style.display = 'inline';
        bdiv.style.display = 'inline';
    }
    var canvas = document.getElementById('hist_canvas');
    if(canvas.width != canvas.clientWidth || canvas.height != canvas.clientHeight) {
        canvas.width = canvas.clientWidth;
        canvas.height = canvas.clientHeight;
    }
    var ctx = canvas.getContext("2d");
    var tr = getHistTrans(); 
    var sp = getStretchParams();
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    var hist = Array.from(g_histogram,(x) => Math.log(x+1));
    var maxv=hist.reduce((x,y)=>Math.max(x,y),0) + 1;
    ctx.strokeStyle = "red";
    ctx.setLineDash([]);
    ctx.lineWidth = 1;
    
    ctx.beginPath();
    for(var i=0;i<hist.length;i++) {
        let x = i * 1.0 / hist.length;
        let y = hist[i]/maxv;
        let xp = tr.getX(x);
        let yp = tr.getY(y);
        if(i==0)
            ctx.moveTo(xp,yp);
        else
            ctx.lineTo(xp,yp);
    }
    ctx.stroke();
    ctx.setLineDash([5,10]);
    ctx.strokeStyle = "#FFC000";
    ctx.lineWidth=3;
    ctx.beginPath();
    var black_p = tr.getX(tr.x_black); 
    var white_p = tr.getX(tr.x_white); 
    var med_p = tr.getX(tr.x_med); 
    var txt_p = tr.getY(1.0);
    ctx.moveTo(black_p,0)
    ctx.lineTo(black_p,canvas.height);
    ctx.moveTo(white_p,0)
    ctx.lineTo(white_p,canvas.height);
    ctx.moveTo(med_p,0)
    ctx.lineTo(med_p,canvas.height);
    ctx.stroke();
    ctx.beginPath();
    ctx.textBaseline = 'bottom';
    ctx.textAlign = 'center'
    ctx.fillStyle = '#FFC000'
    ctx.font = '4mm Sans';
    ctx.textAlign = 'right'
    ctx.fillText('0',black_p,txt_p);
    ctx.textAlign = 'center'
    ctx.fillText('½',med_p,txt_p);
    ctx.textAlign = 'left'
    ctx.fillText('1',white_p,txt_p);
    ctx.stroke();
    ctx.lineWidth=1;
    ctx.setLineDash([2,4]);
    ctx.beginPath();
    var a_factor = gammaToAsinh(1/sp.gamma);
    var scale = 1/Math.asinh(a_factor);
    for(var i=0;i<=128;i++) {
        let xv = i / 128.0;
        //let yv = Math.pow(xv,1/sp.gamma);
        let yv = Math.asinh(xv * a_factor) * scale;
        let xp = tr.getX(tr.x_black * (1-xv) + tr.x_white*xv);
        let yp = tr.getY(yv);
        if(i == 0)
            ctx.moveTo(xp,yp);
        else
            ctx.lineTo(xp,yp);
    }
    ctx.stroke();
    if(g_hist_move_line_id!=-1) {
        ctx.lineWidth=2;
        ctx.beginPath();
        ctx.setLineDash([3,3]);
        ctx.strokeStyle = "green";
        let line_x = tr.getX(g_hist_move_line_pos);
        ctx.moveTo(line_x,0)
        ctx.lineTo(line_x,canvas.height);
        ctx.stroke();
    }
}

function updatePushEvents(e) {
    var stats = JSON.parse(e);
    if(stats.type == 'stats') { 
        document.getElementById('stacking_ui_stats').innerHTML = stats.stacked + '/' + stats.missed + '/' + stats.dropped;
        g_since_saved_s = stats.since_saved_s;
        if(stats.histogramm.length == 0)
            g_histogram = null;
        else {
            g_histogram = stats.histogramm;
            updateHistogram();
        }
    }
    else if(stats.type == 'error') {
        document.getElementById('error_notification').style.display='inline';
        g_error_messages[stats.source] = stats.message;
    }
    else if(stats.type == 'mount') {
        document.getElementById('mount_ra').innerHTML = stats.ra;
        document.getElementById('mount_dec').innerHTML = stats.dec;
        document.getElementById('mount_cfg_ra').innerHTML = stats.ra;
        document.getElementById('mount_cfg_dec').innerHTML = stats.dec;
    }
}

function showErrorNotification()
{
    var msg = '';
    for(key in g_error_messages) {
        if(msg!='')
            msg += '<br>';
        msg += key + ": " + g_error_messages[key];
    }
    g_error_messages = {};
    document.getElementById('error_notification').style.display='none';
    showError(msg);
}

function loadFormats()
{
    document.getElementById('open_camera').style.display='none';
    document.getElementById('std_controls').style.display='inline';
    document.getElementById('zoom_controls').style.display='inline';
    showConfig(true);
    restCall('get','/api/camera/formats', null, (formats)=> {
        var sel = document.getElementById('stream_format')
        for(var i=sel.length-1;i>=0;i--) {
            sel.options.remove(i);
        }
        var saved_sel = loadSavedInputValue('stream_format');
        var matched = false;
        for(var i=0;i<formats.length;i++) {
            var option = document.createElement("option");
            option.text = formats[i].format_id;
            option.value = formats[i].format_id;
            if(formats[i].format_id == saved_sel)
                matched = true;
            sel.add(option)
        }
        if(matched)
            sel.value = saved_sel;
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
        else if(ctl.type == 'select') {
            control_str +=`<td><select class="val_input" id="control_opt_${ctl.option_id}" onchange="enableNumControl('${ctl.option_id}',true);" >`;
            for(var j=0;j<ctl.options.length;j++) {
                var selected = j == ctl.cur ? 'selected ' : '';
                control_str += `<option value="${j}" ${selected} >${ctl.options[j]}</option>`;
            }
            control_str += `</select></td>`;
            control_str += `<td><button disabled id="control_button_${ctl.option_id}" onclick="updateNumControl('${ctl.option_id}');" >set</button></td>`;
            control_str += '<td>&nbsp;</td>';
            control_str += '<td>&nbsp;</td>';
            console.log(`Adding ${ctl.option_id}`)
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
    var selected = loadSavedInputValue(elemid);
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
    if(found)
        sel.value = selected;
}

function saveCalibValue(el)
{
    saveInputValue(el.id);
}


function updateCalibFrames(data)
{
    updateCalibFramesById(data,'stack_darks');
    updateCalibFramesById(data,'stack_flats');
    updateCalibFramesById(data,'stack_dark_flats');
    updateCalibTable(data)
}

function updateCalibTable(data)
{
    var bdy = '';
    for(var i=0;i<data.length;i++) {
        var fr = data[i];
        var date = fr.date.split(' ')[0];
        bdy += "<tr>";
        bdy += `<td>${fr.id}</td><td>${fr.width}</td><td>${fr.height}</td><td>${date}</td>`;
        bdy += `<td><button onclick="deleteCalibFrame('${fr.id}');">del</button></td>`;
        bdy += "</tr>\n";
    }
    document.getElementById('calib_table').innerHTML = bdy;
}

function deleteCalibFrame(frame_id)
{
    requestConfirmation(`Are you shure you want to delete ${frame_id}`,()=> {
        restCall('delete',`/api/stacker/calibration/${frame_id}`,null,(e)=>{ 
            loadCalibFrames(); 
        });
    });
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
    var video = document.getElementById('live_stream_video');
    if(name == 'none') {
        if(video) {
            video.src = '/media/invalid.jpeg';
        }
        document.getElementById('video').innerHTML = '';
    }
    else {
        if(video)
            video.src = `/api/video/${name}`;
        else
            document.getElementById('video').innerHTML = `<div id="live_stream_video_div"><img id="live_stream_video" alt="streaming video" src="/api/video/${name}" /></div>`;
        onResize(null);
    }
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
    var mjpeg = document.getElementById('thumb_stream_video');
    if(v) {
        if(mjpeg)
            mjpeg.src = "/api/video/live";
        else
            thumb_video.innerHTML = `<img id="thumb_stream_video" alt="streaming video" src="/api/video/live" />`;
        thumb.style.display = 'inline';
        live_button.innerHTML = 'X';
        onResize(null);
    }
    else {
        if(mjpeg)
            mjpeg.src = '/media/invalid.jpeg';
        else 
            thumb_video.innerHTML ='';
        thumb.style.display = 'none';
        live_button.innerHTML = 'L';
    }
}

function showLiveVideo()
{
    showStream('live');
}

function setVideoScale(video,video_div,size)
{
    video_div.style.width  = size[0]+ 'px';
    video_div.style.height = size[1] + 'px';
    video_div.style.marginLeft = size[2] + 'px';
    video_div.style.marginTop  = size[3] + 'px';
   
    if(global_zoom == 1) {
        video_div.style.overflowX = 'hidden';
        video_div.style.overflowY = 'hidden';
    }
    else {
        video_div.style.overflowX = 'scroll';
        video_div.style.overflowY = 'scroll';
    }
    video.style.width  = size[0] * global_zoom + 'px';
    video.style.height = size[1] * global_zoom + 'px';
    var dx = size[0] * (global_zoom - 1) / 2;
    var dy = size[1] * (global_zoom - 1) / 2;
    if(video_div.scroll) {
        video_div.scroll(dx,dy);
    }
    else {
        video_div.scrollLeft = dx;
        video_div.scrollTop = dy;
    }
}

function onResize(ev)
{
    var video = document.getElementById('live_stream_video');
    var video_div = document.getElementById('live_stream_video_div');
    var size = calcImgSize();
    if(video) {
        setVideoScale(video,video_div,size);
    }
    var thumb = document.getElementById('thumb_stream_video')
    if(thumb) {
        thumb.style.width = Math.round(size[0] / 4) + 'px';
        thumb.style.height = Math.round(size[1] / 4) + 'px';
        thumb.style.marginRight = '0px';
        thumb.style.marginTop  = '0px';
    }
    var solved = document.getElementById('solver_result_image');
    var solved_div = document.getElementById('solver_result_image_div');
    if(solved) {
        setVideoScale(solved,solved_div,size);
    }
    updateHistogram();
}

function setFontSizeForElements(tags,fs)
{
    for(var i=0;i<tags.length;i++)
        tags[i].style.fontSize = fs;
}

function setFontSizeForTag(tag,fs)
{
    setFontSizeForElements(document.getElementsByTagName(tag));
}

function setFontSizeForClass(klass,fs)
{
    setFontSizeForElements(document.getElementsByClassName(klass));
}

function updateFontSize()
{
    var delta = parseInt(document.getElementById('font_size_ctl').value);
    var f5 = `${5+delta}mm`;
    var f4 = `${4+delta}mm`;
    var body = document.body;
    body.style.fontSize = f5;
    setFontSizeForTag('TR',f5);
    setFontSizeForTag('BUTTON',f5);
    setFontSizeForTag('SELECT',f4);
    setFontSizeForClass('comp_but',f5);
    setFontSizeForClass('incdec_but',f5);
    setFontSizeForClass('hctl',f4);
    var inp = document.getElementsByTagName('INPUT');
    for(var i=0;i<inp.length;i++){ 
        let el = inp[i];
        console.log(el.type);
        if(el.type == 'number' || el.type == 'text')
            el.style.fontSize = f4;
        
    }
}

function stopStream()
{
    restCall('post','/api/camera/stream',{op:'stop'},(r)=>{
        changeStackerStatus('closed');
        showStream('none');
        showConfig(true);
    });
}

function startStream()
{
    var format_id = document.getElementById('stream_format').value;
    var max_fr = parseFloat(document.getElementById('stream_max_framerate').value);
    if(isNaN(max_fr))
        max_fr = 0;
    restCall('post','/api/camera/stream',{op:'start',format_id:format_id, max_framerate: max_fr },(r)=>{
        saveInputValue('stream_format');
        global_width = r.width;
        global_height = r.height;
        global_bin = r.bin;
        showLiveVideo();
        changeStackerStatus('idle');
        recalcFOV();
    });
    showConfig(false);
}

function showMount(disp)
{
    document.getElementById('mount_controls').style.display = disp ? 'block' : 'none';
}

function noslew() { 
    slew('',null);
}

function slew(direction)
{
    restCall('post','/api/mount/slew',{"direction":direction,"speed" : g_slew_speed },(e) => {});
}

function openConfigTab(name)
{
    var tabCtl = {
        config: showConfig,
        stack : showStack,
        pp:     showPP,
        sharp:  showSharp,
        solve:  showSolve,
        mount:  showMount,
    };
    for(var n in tabCtl) {
        if(n != name)
            tabCtl[n](false);
    }
    tabCtl[name](true);
}

function showStack(v)
{
    if(v) {
        loadCalibFrames();
    }
	document.getElementById('stack').style.display = v ? 'inline' : 'none';
}

function showSharp(v)
{
	document.getElementById('sharp_control').style.display = v ? 'inline' : 'none';
}

function updateShrapCtl(base_id,digits,value)
{
    var sv=parseFloat(value).toFixed(digits);
    document.getElementById("out_" + base_id).value = sv;
    if(g_stacker_status == 'stacking' || g_stacker_status == 'paused') { 
        updateSharpen();
    }
}

function showPP(v)
{
	document.getElementById('pp_control').style.display = v ? 'inline' : 'none';
    if(v) {
        restCall('get','/data/stretch.json',null,getAutoStretchState,setDefaultAutoStretchState);
    }
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

function getStartDelay()
{
    var val = parseFloat(document.getElementById('start_delay').value);
    if(isNaN(val) || val < 0)
        return 0.0;
    if(val > 10)
        return 10;
    return val;
}

function resumeStack()
{
    var delay = getStartDelay();
    if(delay == 0) {
        ctlStack('resume');
    }
    else {
        showNotification('Resuming',delay,true,()=>{ctlStack('resume');});
    }
}

function ctlStack(status,confirm_may_be_needed=true)
{
    if(status == 'save') {
        showNotification('Saving',1,false);
    }
    if(status == 'cancel' && g_since_saved_s >= 60 && confirm_may_be_needed) {
        requestConfirmation(`You have collected ${g_since_saved_s.toFixed(1)}s of unsaved data. Are you sure you want to discard it?`,()=>{
            ctlStack('cancel',false);
        });
        return;
    }
    restCall('post','/api/stacker/control',{operation:status},(s)=> {
        if(status == 'cancel') {
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


function updatePPSliders(data)
{
    g_stretch.cut = data.cut;
    g_stretch.gain = data.gain;
    g_stretch.gamma = data.gamma;
    document.getElementById('p_gain').innerHTML = data.gain.toFixed(2);
    document.getElementById('p_cut').innerHTML = data.cut.toFixed(3);
    document.getElementById('p_gamma').innerHTML = data.gamma.toFixed(1);
    updatePP();
}

function setDefaultAutoStretchState()
{
    document.getElementById('stack_auto_stretch').checked=true;
    setDefaultPPSliders();
}

function getAutoStretchState(e)
{
    var as = 'auto_stretch' in e ? e.auto_stretch : true;
    document.getElementById('stack_auto_stretch').checked=as;
    document.getElementById('dynamic_parameters').style.display= as ? 'none' : 'inline';
    if(!as) {
        updatePPSliders(e);
    }
    g_show_hist = !as;
    updateHistogram();
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
    g_show_hist = !auto_pp;
    updateHistogram();
    if(auto_pp) {
        document.getElementById('dynamic_parameters').style.display='none';
        updatePP();
        return;
    }
    document.getElementById('dynamic_parameters').style.display='inline';
    restCall('get','/data/stretch.json',null,updatePPSliders,setDefaultPPSliders);
}


function configSharpenning(config)
{
    config.deconv_sig = parseFloat(getVal("deconv_sigma")); 
    config.deconv_iters = parseInt(getVal("deconv_iters"));
    config.unsharp_sig = parseFloat(getVal("unsharp_sigma"));
    config.unsharp_strength = parseFloat(getVal("unsharp_strenght"));
}

function updateSharpen()
{
    g_sharpen_sent = new Date().getTime();
    if(g_sharpen_delay)
        return;
    updateSharpenCheckSetTimeout(2100.0);
}

function updateFltSlider(name)
{
    var val = document.getElementById('stack_' + name).value;
    if(name == 'filters_avg_brightness_sigma' && parseFloat(val) == 5)
        val = 'all';
    document.getElementById('out_' + name).value = val;
}

function updateSharpenCheckSetTimeout(to)
{
    g_sharpen_delay = true;
    setTimeout(()=> { 
            g_sharpen_delay = false;
            updateSharpenCheck();
    },to);
}

function updateSharpenCheck()
{
    var now = new Date().getTime();
    var reminder = now - g_sharpen_sent;
    if(reminder >= 2000) {
        configSharpenning(config);
        console.log("Sending sharpen config " + JSON.stringify(config));
        restCall('post','/api/stacker/sharpen',config,(e)=>{});
        return;
    }
    updateSharpenCheckSetTimeout(2100 - reminder);
}

function updatePP()
{

    var config={
        auto_stretch:       getBVal("auto_stretch"),
        stretch_low:        g_stretch.cut,
        stretch_high:       g_stretch.gain,
        stretch_gamma:      g_stretch.gamma
    };
    restCall('post','/api/stacker/stretch',config,(e)=>{
    });
    updateHistogram();
}

function startStack()
{
    var type = getVal('type');
    var darks = null,flats = null, dark_flats = null;
    if(type != 'calibration') {
        darks = getVal('darks');
        if(darks == 'N/A') {
            darks = null;
        }
        flats = getVal('flats');
        if(flats == 'N/A') {
            flats = null;
        }
        dark_flats = getVal('dark_flats');
        if(dark_flats == 'N/A') {
            dark_flats = null;
        }
    }
    var ra=null,de=null,lat=null,lon=null,synthetic_exposure_mpl=1;
    var field_derotation=false,image_flip=false,remove_satellites=false,remove_gradient=false;
    var remove_hot_pixels=getBVal('remove_hot_pixels');
    if(type == 'dso') {
        ra=parseRA(getVal("ra"));
        de=parseDEC(getVal("de"));
        lat = parseFloat(getVal("lat"));
        lon = parseFloat(getVal("lon"));
        synthetic_exposure_mpl = parseFloat(getVal("synthetic_exposure_mpl"));
        field_derotation = getBVal("field_derotation");
        image_flip =         getBVal("image_flip");
        remove_satellites =  getBVal("remove_satellites");
        remove_gradient =    getBVal("remove_gradient");
    }
    var filters = {
        remove_first:false,
        min_stat_size:10,
        dynamic_delay: 2,
        sharpness_percentile:100,
        reg_score_percentile:100,
        avg_brightness_sigma:-1,
    };
    if(type != 'calibration') {
        if(type == 'planetary') {
            remove_hot_pixels=false;
            filters.remove_first = getBVal('filters_remove_first');
        }
        filters.dynamic_delay = parseInt(getVal('filters_dynamic_delay'));
        filters.min_stat_size = parseInt(getVal('filters_min_stat_size'));
        filters.sharpness_percentile = parseFloat(getVal('filters_sharpness_percentile'));
        filters.reg_score_percentile = parseFloat(getVal('filters_reg_score_percentile'));
        filters.avg_brightness_sigma = parseFloat(getVal('filters_avg_brightness_sigma'));
        if(filters.avg_brightness_sigma >= 5.0)
            filters.avg_brightness_sigma = -1;
    }
    var delay = getStartDelay();
    if(type == 'dynamic')
        delay = 0;
    if(isNaN(synthetic_exposure_mpl))
        synthetic_exposure_mpl = 1
    var rollback_on_pause = delay > 0;
    if(field_derotation && type=='dso') {
        if(isNaN(lat) || isNaN(lon)) {
            showError('Need Geolocation for derotation support\n(see settings-general menu)');
            return;
        }
        if(ra==null || de==null) {
            showError('Need RA, DE for derotation');
            return;
        }
    }
    if(isNaN(lat))
        lat = 0;
    if(isNaN(lon))
        lon = 0;

    var name = getVal('name').trim();
    var obj = getVal('object').trim();

    if(toRaDecCoord(obj) != null) {
        obj = '';
    }

    if(type == 'dso') {
        if(name == '')
            name = obj;
        else if(obj != '')
            name = name + '_' + obj;
    }

    name = name.replace(/ /g,'_');
    const name_pat = /^[A-Za-z0-9_.\-]*$/g;
    if(!name.match(name_pat)) {
        showError('Name and Object should contain only English letters, digits, "_", "-" and "."')
        return;
    }
    if(type == 'calibration' && name=='') {
        showError('Provide name for calibration frame');
        return;
    }

    var config={
        name:               name,
        save_data:          getBVal("save_data"),
        field_derotation:   field_derotation,
        synthetic_exposure_mpl: synthetic_exposure_mpl,
        image_flip:         image_flip,
        remove_satellites:  remove_satellites,
        remove_gradient:    remove_gradient,
        remove_hot_pixels:  remove_hot_pixels,
        rollback_on_pause:  rollback_on_pause,
        auto_stretch:       getBVal("auto_stretch"),
        stretch_low:        g_stretch.cut,
        stretch_high:       g_stretch.gain,
        stretch_gamma:      g_stretch.gamma,
        type:               type,
        location : {
            lat:            lat,
            lon:            lon,
        },
        target: {
            ra:             ra,
            de:             de,
        },
        filters : filters,
        darks : darks,
        flats : flats,
        dark_flats : dark_flats
    };

    configSharpenning(config);

    var doStackFunc = ()=> {
        restCall('post','/api/stacker/start',config,(e)=>{
            changeStackerStatus('stacking');
            showStack(false);
            if(delay > 0 && type!='calibration') {
                showThumbLive(true);
            }
        });
    };
    if(delay == 0) {
        document.getElementById('stack').style.display = 'none';
        showNotification('Starting Stacking',1,false);
        doStackFunc();
    }
    else {
        document.getElementById('stack').style.display = 'none';
        showNotification('Stacking',delay,true,doStackFunc);
    }
    var ws = parseFloat(document.getElementById('white_delay').value);
    ws = isNaN(ws) ? 0 : ws;
    if(ws > 0 && delay > 0 && type=='calibration') {
        setTimeout(()=> { ctlStack('pause')},(delay + ws) * 1000);
        setTimeout(()=> { document.getElementById('white_screen').style.display='none' },(delay *2 + ws) * 1000);
        document.getElementById('white_screen').style.display='block';
    }
}

function updateRADE(name)
{
    updateRADEFor(name,'stack');
    updateRADEFor(name,'solver');
    document.getElementById('solver_object').value = name;
    document.getElementById('mount_object').value = name;
    saveInputValue('solver_object');
}
function updateSolverRADE(name)
{
    updateRADEFor(name,'solver');
    updateRADEFor(name,'stack');
    document.getElementById('stack_object').value = name;
    document.getElementById('mount_object').value = name;
    saveInputValue('stack_object')
}

function updateMountRADE(name)
{
    updateRADEFor(name,'solver');
    updateRADEFor(name,'stack');
    document.getElementById('stack_object').value = name;
    document.getElementById('solver_object').value = name;
    saveInputValue('stack_object')
}


function setDisplayStyle(arr,val)
{
    for(var i=0;i<arr.length;i++)
        arr[i].style.display=val;
}


function toRaDecCoord(name)
{
    let radec = name.split(',')
    if(radec.length != 2)
        return null;
    let ra = radec[0].trim();
    let dec = radec[1].trim();
    ra  = parseRA(ra);
    dec = parseDEC(dec);
    if(ra == null || dec == null)
        return null;
    return [formatRA(ra/15),formatDEC(dec)];
}

function notifySunWarning()
{
    var el = document.getElementById('mount_sun_allow');
    if(el.checked) {
        el.checked = false;
        requestConfirmation("Are you sure you want to allow GoTo Sun? Make sure you use correct and safe equipment!",()=> {
            el.checked = true;
        });
    }
}

function sunIsAllowed()
{
    return document.getElementById('mount_sun_allow').checked;
}

function updateRADEFor(name,target_id)
{
    var coord=['','']
    name = name.toUpperCase().trim();
    var sync = document.getElementsByClassName('mount_sync');
    var no_sync = document.getElementsByClassName('mount_no_sync');
    let do_sync = true;
    if(name in jsdb) {
        coord = jsdb[name];
    }
    else if(isSolarSystemObject(name) && (name != 'SUN' || sunIsAllowed())) {
        var lat = parseFloat(getVal("lat"));
        var lon = parseFloat(getVal("lon"));
        if(isNaN(lat))
            lat = 0;
        if(isNaN(lon))
            lon = 0;
        coord = getSolarSystemObject(name,lat,lon);
    }
    else if(toRaDecCoord(name)) {
        coord = toRaDecCoord(name);
    }
    else {
        do_sync = false
    }
    if(do_sync) {
        setDisplayStyle(sync,'inline');
        setDisplayStyle(no_sync,'none');
    }
    else {
        setDisplayStyle(no_sync,'inline');
        setDisplayStyle(sync,'none');
    }
    document.getElementById(target_id + '_ra').value = coord[0];
    document.getElementById(target_id + '_de').value = coord[1];
}

function mountGoToSelected()
{
    var name = document.getElementById('mount_object').value;
    // make sure we have latest RA/DEC
    updateMountRADE(name);
    name = name.toUpperCase().trim();
    var track = 'sidereal';
    if(name == 'SUN')
        track = 'solar';
    else if(name == 'MOON')
        track = 'lunar'
    var mode_el = document.getElementById('mount_tracking_mode');
    if(mode_el.value != track) {
        mode_el.value = track;
        mountSetTrackingMode();
    }
    var ra = document.getElementById('solver_ra').value;
    var de = document.getElementById('solver_de').value;
    mountGoTo(ra,de);
}

function mountSyncSelected()
{
    var ra = document.getElementById('solver_ra').value;
    var de = document.getElementById('solver_de').value;
    restCall('post','/api/mount/sync',{'ra':ra,'dec':de},(e)=>{
        setTimeout(update_mnt,300);
    }) 
}

function manualSyncPressStart()
{
    g_sync_counter ++;
    let cur_counter = g_sync_counter + 0;
    setTimeout(()=> {
        if(cur_counter == g_sync_counter) {
            mountSyncSelected();
            showNotification('Sync Command Sent');
        }
        else {
            showNotification('Press on sync button for 2 second to sync');
        }
    },2000);
}
function manualSyncPressEnd()
{
    g_sync_counter++;
}

function recalcFOV()
{
    try {
        var pixel_size = parseFloat(document.getElementById('solver_pixel_size').value);
        var FL = parseFloat(document.getElementById('solver_focal_length').value);
        if(isNaN(pixel_size) || isNaN(FL))
            return;
        var fov = global_height * global_bin * pixel_size * 1e-3 / FL * 180 / 3.14159;
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
    g_solving_ui_open = show;
    document.getElementById('plate_solve_div').style.display = show ? 'inline' : 'none';
    document.getElementById('solver_auto_restart').checked = false;
    if(!show) {
        document.getElementById('solver_result').style.display = 'none';
    }
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


function plateSolveWithOpt(background,result_callback,sync_op)
{
    var req;
    try {
        var ra,de,mount_ra,mount_de,fov,rad,lat,non,timeout;
        var use_mount_pos = g_mount_connected && document.getElementById('use_mount_coordinates').checked;
        ra = parseRA(document.getElementById('solver_ra').value);
        de = parseDEC(document.getElementById('solver_de').value);
        if(use_mount_pos) {
            mount_ra = parseRA(document.getElementById("mount_ra").innerHTML);
            mount_de = parseDEC(document.getElementById("mount_dec").innerHTML);
            if(sync_op) {
                ra = mount_ra;
                de = mount_de;
            }
        }
        else {
            mount_ra = ra;
            mount_de = de;
        }
        fov = parseFloat(document.getElementById('solver_fov').value);
        rad = parseFloat(document.getElementById('solver_radius').value);
        timeout = parseFloat(document.getElementById('solver_timeout').value);
        lat = parseFloat(getVal("lat"));
        lon = parseFloat(getVal("lon"));
        if(isNaN(lat))
            lat = null;
        if(isNaN(lon))
            lon = null;
        if(isNaN(ra) || ra==null || isNaN(de) || de==null || isNaN(fov) || isNaN(rad) || isNaN(timeout)) {
            throw new Error('RA, DE, Radius, Timeout and FOV are required');
        }
        req = {
            "fov" : fov,
            "ra"  : ra,
            "de"  : de,
            "mount_ra" : mount_ra,
            "mount_de" : mount_de,
            "rad" : rad,
            "lat" : lat,
            "lon" : lon,
            "timeout" : timeout
        };
        JSON.stringify(req); // check it is OK
    }
    catch(e) {
        if(background) {
            startSolvingIfNeeded();
        }
        else {
            showError('Invalid Inputs' + e)
        }
        return;
    }
    if(!background) {
        document.getElementById('solver_config').style.display = 'none';
        document.getElementById('solver_status_div').style.display = 'inline';
        document.getElementById('solver_status').innerHTML = 'Waiting...';
        document.getElementById('solver_result').style.display = 'none';
    }
    g_solving = true;
    restCall('post','/api/plate_solver',req,(r) => { solveResult(r,result_callback); })
}

function plateSolve()
{
    plateSolveWithOpt(false,null,false);
}

function resolveOnTimer()
{
    plateSolveWithOpt(true,null,false);
}

function startSolvingIfNeeded()
{
    var on = document.getElementById('solver_auto_restart').checked;
    if(on && g_solving_ui_open == true && g_solving == false) {
        g_solving = true;
        var timeout = parseFloat(document.getElementById('restart_solver_timeout').value);
        if(isNaN(timeout) || timeout < 1.0)
            timeout = 1;
        setTimeout(resolveOnTimer,1000*timeout);
    }
}

function formatAngle(deg,as_hours)
{
    var sep;
    var sep2;
    if(as_hours) {
        deg = deg / 15;
        sep = ":";
        sep2 = ":";
    }
    else {
        sep = "\u00b0";
        sep2 = "'";
    }
	var direction = deg >= 0 ? '+' : '-';
	deg = Math.abs(deg);
	var ideg = Math.floor(deg);
	var min = 60*(deg - ideg);
	var imin = Math.floor(min);
	var sec = 60*(min - imin);
    var msg = direction + ideg + sep + imin + sep2;
    if(as_hours)
	    msg += Math.floor(sec);
    return msg;
}

function formatTableRow(a)
{
    var res = '<tr>';
    for(var i=0;i<a.length;i++) {
        res += '<td class="solver_results">' + a[i] + '</td>';
    }
    res += '</tr>\n';
    return res;
}

function getFOVs()
{
    var vfov = parseFloat(document.getElementById('solver_fov').value);
    var hfov = global_width * vfov / global_height;
    var dfov = Math.sqrt(vfov*vfov + hfov*hfov);
    return {
        d:dfov,
        v:vfov,
        h:hfov,
    };
}

function updateSolverTable()
{
    if(g_solver_result == null)
        return;
    var v=g_solver_result;
    if(v.solved) {
        g_solver_result = v;
        status = 'Distance to target ' + v.distance_to_target.toFixed(2) + '°';
        var fovs = getFOVs();
        console.log(`${JSON.stringify(fovs)} dist=${v.distance_to_target}`)
        if(v.distance_to_target*2 > fovs.v || v.distance_to_target*2 > fovs.h) {
            let dist_screens = (v.distance_to_target / fovs.d).toFixed(1);
            status += ` or ${dist_screens} diagonals`;
        }
        var opt = document.getElementById('solver_units').value;
        var warn = '';
        var v0,v1;
        var u0,u1;
        if(opt.substring(0,2) == 'az') {
            u0 = '∆ALT';
            u1 = '∆AZ';
            if(v.delta_az) {
                if(opt=='az_dd_dd') {
                    v0 = v.delta_alt.toFixed(2) + "\u00b0";
                    v1 = v.delta_az.toFixed(2) + "\u00b0";
                }
                else {
                    v0 = formatAngle(v.delta_alt,false);
                    v1 = formatAngle(v.delta_az,false);
                }
            }
            else {
                warn = 'for ∆Alt/∆AZ provide geolocation in settings';
                v0='N/A';
                v1='N/A';
            }
        }
        else {
            u0 = '∆RA';
            u1 = '∆DEC'
            if(opt=='rd_dd_dd') {
                v0 = v.delta_ra.toFixed(2) + "\u00b0";
                v1 = v.delta_de.toFixed(2) + "\u00b0";
            }
            else {
                v0 = formatAngle(v.delta_ra,(opt == 'rd_hm_dm'));
                v1 = formatAngle(v.delta_de,false);
            }
        }
        status += '<table class="solver_results" style="border:1px solid; table-layout: fixed; width:80%; ">'
        status += formatTableRow([u0,v0]);
        status += formatTableRow([u1,v1]);
        status += '</table>'
        status += warn;
    }
    else {
        status = v.error;
    }
    document.getElementById('solver_status').innerHTML = status;
}
function formatRA(RA)
{
    let rh = Math.floor(RA);
    let rs = Math.floor(RA*3600) % 3600;
    let rm = Math.floor(rs / 60);
    rs = rs % 60;
    return `${rh}:${rm}:${rs}`;
}

function formatDEC(DEC)
{
    let s = 1;
    if(DEC < 0) {
        s=-1;
        DEC=-DEC;
    }
    let dd = Math.floor(DEC);
    let ds = Math.floor(DEC*3600) % 3600;
    let dm = Math.floor(ds / 60);
    ds = ds % 60;
    dd = dd * s;
    return `${dd}:${dm}:${ds}`
}

function syncSolve(r,call_goto)
{
    var ra_h = r.center_ra / 15;
    var de_d = r.center_de;
    restCall('post','/api/mount/sync',{'ra':formatRA(ra_h),'dec':formatDEC(de_d)},(r)=>{
        setTimeout(update_mnt,300);
        if(call_goto) {
            mountGoToSelected();
        }
    });
}

function solveAndSync()
{
    plateSolveWithOpt(false,(v)=>{syncSolve(v,false);},true);
}

function solveSyncAndGoTo()
{
    plateSolveWithOpt(false,(v)=>{syncSolve(v,true);},true);
}



function solveResult(v,callback)
{
    g_solving = false;
    g_solver_result = null;
    if(!g_solving_ui_open)
        return;
    g_solver_result = v;
    updateSolverTable();
    var jpeg = v.solved ? v.result_image + '?' + Date.now() : null;
    if(jpeg) {
        document.getElementById('solver_result_image').src = jpeg;
    }
    document.getElementById('solver_result').style.display = jpeg ? 'inline' : 'none';
    if(v.solved && callback) {
        callback(v);
    }
    startSolvingIfNeeded();
}

function solverRestart()
{
    document.getElementById('solver_status_div').style.display='none';
    document.getElementById('solver_config').style.display='inline';
    document.getElementById('solver_result').style.display = 'none';
    document.getElementById('solver_auto_restart').checked = false;
}

function selectAstapConfig()
{
    restCall("get","/api/astap_db",null,onAstapVisible);
}

function onAstapVisible(st)
{
    if(!st.has_db) {
        document.getElementById('astap_ready_list').innerHTML = 'No Star DB for ASTAP';
    }
    else {
        document.getElementById('astap_ready_list').innerHTML = st.db.join(', ');
    }
    console.log(st)
    var sel = document.getElementById('astap_download_db');
    console.log('Selected = ' + sel.value);
    for(var opt=0;opt < sel.options.length;opt++) {
        setDownloadURL('');
        if(st.db.indexOf(sel.options[opt].value)!=-1) {
            sel.options[opt].disabled=true;
        }
        else {
            if(!sel.value) {
                sel.options[opt].selected = true;
            }
        }
    }
    setDownloadURL(sel.value);
    updateDownloadStatus(st.downloading)
}

function updateDownloadProgress(data)
{
    var msg;
    var st = JSON.parse(data);
    if(st.completed) {
        updateDownloadStatus(false);
        if(st.success)
            msg = 'Done';
        else
            msg = 'Failed: ' + st.error;
        selectAstapConfig();
    }
    else {
        msg = `Downloading ${st.last_file}, ${st.downloaded}/${st.expected}`;
    }
    document.getElementById('download_status').innerHTML = msg;
}

function downloadStart()
{
    var url = document.getElementById('astap_download_url').value;
    if(url == '') {
        showError('URL is empty');
        return;
    }
    var el = document.getElementById('astap_download_db');
    var db = el.value;
    restCall('post','/api/astap_db/download',{"db": db, "url":url},(r) =>{
        updateDownloadStatus(true);
    });
}

function downloadStop()
{
    restCall('post','/api/astap_db/cancel',{},(r)=> {
        updateDownloadStatus(false);
    });

}

function updateDownloadStatus(active)
{
    var dstart = document.getElementById('download_start');
    var dstop = document.getElementById('download_stop');
    dstart.style.display = active ? 'none':'inline';
    dstop.style.display = !active ? 'none':'inline';
    if(active) {
        if(!global_download_progress) {
            global_download_progress = new EventSource("/api/astap_db/progress");
            global_download_progress.onmessage = (e) => {
                updateDownloadProgress(e.data);
            };
        }
    }
    else {
        if(global_download_progress) {
            global_download_progress.close();
            global_download_progress = null;
        }
    }
}

function switchASTAPURL()
{
    var sel = document.getElementById('astap_download_db');
    setDownloadURL(sel.value);
    return true;
}

function setDownloadURL(db_id)
{
    var url = ''
    if(db_id != '') {
        var extra = '';
        if(db_id == 'w08')
            extra = '_mag08_astap';
        var base = 'https://downloads.sourceforge.net/project/astap-program/star_databases/';
        //var base = 'http://127.0.0.1:8080/media/';
        url = `${base}/${db_id}_star_database${extra}.zip`;
    }
    document.getElementById('astap_download_url').value = url;
}


function mountStart(name)
{
    var name = document.getElementById('mount_driver_name').value;
    var opt = document.getElementById('mount_driver_option').value;
    restCall('post','/api/mount/start',{'driver':name,'option':opt},(e)=>{
        update_mnt();
    });
}

function waitMountConnUpdate(data) {
    var button = document.getElementById('mount_connect_button');
    if(data.connected) {
        updateMountFunc(data);
        button.disabled = false;
    }
    else if(g_wait_connected > 0) {
        g_wait_connected --;
        setTimeout(()=>{
            restCall('get','/api/mount/config',null,
            (data)=>{
                waitMountConnUpdate(data);
            },
            (e) => {
                button.disabled = false;
                showError(e);
            })
        },500);
    }
    else {
        button.disabled = false;
        showError("Connection Failed"); 
    }
}

function mountConnect()
{
    g_wait_connected = 10;
    document.getElementById('mount_connect_button').disabled = true;
    restCall('post','/api/mount/connect',{},(e) => {
        waitMountConnUpdate({"connected":false})
    });
}

function mountGoTo(ra,dec)
{
    restCall('post','/api/mount/goto',{'ra':ra,'dec':dec},(e)=>{}) 
}

function loadMountProtocols(data)
{
    var current = data.proto.current;
    var types = document.getElementById('mount_connection_type');
    for(var i=types.options.length - 1; i>= 0 ;i--) {
        types.options.remove(i);
    }
    var addr = '';
    if(data.proto.inet.supported) {
        var op = document.createElement('option');
        op.text = 'Network';
        op.value = 'inet';
        op.selected = current == 'inet';
        if(op.selected)
            addr = data.proto.inet.connection;
        types.add(op)
    }
    if(data.proto.serial.supported) {
        var op = document.createElement('option');
        op.text = 'Serial';
        op.value = 'serial';
        op.selected = current == 'serial';
        if(op.selected)
            addr = data.proto.serial.connection;
        types.add(op)
    }
    document.getElementById('mount_connection_address').value = addr;
    document.getElementById('mount_connection_address_set').disabled = true;
}

function mountConnTypeMod()
{
    var name = document.getElementById('mount_connection_type').value;
    restCall('post','/api/mount/connection_proto',{'proto':name},(v) => {
        setTimeout(update_mnt,500);
    });
}

function mountConnStrSet()
{
    var addr = document.getElementById('mount_connection_address').value;
    var proto = document.getElementById('mount_connection_type').value;
    restCall('post','/api/mount/connection_addr',{'proto': proto, 'addr':addr},(v) => {
        setTimeout(update_mnt,500);
    });
}

function mountDriverSelect()
{
    var name = document.getElementById('mount_driver_name').value;
    st = getStoageValue();
    st['mount_driver'] = name;
    setStorageValue(st);
    setSlewEventListeners();
    var opt = g_mount_driver_populated[name];
    var dopt = document.getElementById('mount_driver_option');
    var popt = document.getElementById('mount_driver_option_p');
    if(opt == null) {
        opt = ''
        dopt.disabled = true;
        popt.style.display = 'none';
    }
    else {
        dopt.disabled = false;
        popt.style.display = 'block';
    }
    dopt.value = opt;
}

function updateMountFunc(data)
{
    if(!data.server) {
        document.getElementById('mount_driver_selection').style.display='block';
        if(!g_mount_driver_populated) {
            var driver_options = {}
            var selected_driver = getStoageValue()['mount_driver']
            var drivers = document.getElementById('mount_driver_name');
            for(var i=0;i<data.server_drivers.length;i++) {
                var option = document.createElement("option");
                var name = data.server_drivers[i].name;
                var has_opt = data.server_drivers[i].has_option;
                var driver_opt = data.server_drivers[i].option;
                if(!selected_driver)
                    selected_driver = name;
                driver_options[name] = has_opt ? driver_opt : null;
                option.text  = name;
                option.value = name;
                option.selected = selected_driver == name;
                drivers.add(option);
            }
            g_mount_driver_populated = driver_options;
            mountDriverSelect();
        }
    }
    else {
        document.getElementById('mount_driver_selection').style.display='none';
        if(!data.connected) {
            document.getElementById('mount_connection_options').style.display='';
            loadMountProtocols(data);
        }
        else {
            g_mount_connected = true;
            document.getElementById('mount_connection_options').style.display='none';
            var items_to_enable = document.getElementsByClassName('mount_connected_display_inline');
            for(var i=0;i<items_to_enable.length;i++) {
                items_to_enable[i].style.display='inline';
            }
            var items_to_enable = document.getElementsByClassName('mount_connected_display_block');
            for(var i=0;i<items_to_enable.length;i++) {
                items_to_enable[i].style.display='block';
            }
            loadMountStatus();
        }
    }
}

function toggleRADE(disp)
{
    document.getElementById('mount_ra_de_coord').style.display = disp ? 'inline' : 'none';
}

function mountUpdateSlewSpeedUI(delta)
{
    g_slew_speed += delta;
    if(g_slew_speed < 0)
        g_slew_speed = 0;
    if(g_slew_speed >= g_slew_max)
        g_slew_speed = g_slew_max - 1;
    document.getElementById('mount_slew_speed').innerHTML = `${g_slew_speed + 1}`;
}

function mountResetAlignment()
{
    restCall('post','/api/mount/reset_alignment',{},(e)=>{
        setTimeout(update_mnt,300);
    });
}

function loadMountStatus()
{
    restCall('get','/api/mount/status',null,(data) => {
        g_slew_max = data.slew.max;
        g_slew_speed = data.slew.current;
        console.log(`max=${g_slew_max} cur=${g_slew_speed}`)
        mountUpdateSlewSpeedUI(0);
        document.getElementById("mount_tracking_mode").value = data.tracking_mode;
        document.getElementById("mount_lat").innerHTML = data.lat.toFixed(2);
        document.getElementById("mount_lon").innerHTML = data.lon.toFixed(2);
        document.getElementById('mount_alignment_reset').disabled = data.alignment < 0;
        var alignment_status;
        if(data.alignment < 0)
            alignment_status = "not supported";
        else if(data.alignment == 0)
            alignment_status = "not aligned";
        else if(data.alignment > 10)
            alignment_status = "on multiple points";
        else
            alignment_status = `on ${data.alignment} points`;
        document.getElementById('mount_alignment').innerHTML = alignment_status;
        if(!g_mount_geolocation_updated) {
            var lat = parseFloat(getVal("lat"));
            var lon = parseFloat(getVal("lon"));
            if(isNaN(lat) || isNaN(lon)) {
                showError("Lattitude/Longitude is not defined, moust geolocation is not updated");
                return;
            }
            if(Math.abs(lat - data.lat) > 0.0001 || Math.abs(lon-data.lon) > data.lon) {
                restCall('post','/api/mount/geolocation',{"lat":lat,"lon":lon},(r)=>{
                    g_mount_geolocation_updated = true;
                    update_mnt();
                });
            }
        }
    });
}

function mountSetTrackingMode()
{
    var mode = document.getElementById("mount_tracking_mode").value;
    restCall("post","/api/mount/tracking",{"tracking_mode":mode},(e)=>{});
}

function update_mnt() {
    restCall('get','/api/mount/config',null,updateMountFunc);
}

function selectConfig(cfg)
{
    var cfgs = ['astap','general','camera','profiles','calib','mount'];
    for(var i=0;i<cfgs.length;i++) {
        var obj = document.getElementById('config_tab_' + cfgs[i]);
        if(cfgs[i] == cfg) {
            obj.style.display = 'block';
        }
        else {
            obj.style.display = 'none';
        }
    }
    if(cfg=='astap') {
        selectAstapConfig();
    }
    else if(cfg=='profiles') {
        selectProfile();
    }
    else if(cfg == 'calib') {
        loadCalibFrames();
    }
    if(cfg == 'mount') {
        update_mnt();
    }
}

function getProfiles()
{
    var storage = window.localStorage;
    var data = storage.getItem('ols_profiles');
    if(!data)
        data = [];
    else
        data = JSON.parse(data);
    return data;
}

function saveProfiles(data)
{
    var storage = window.localStorage;
    storage.setItem('ols_profiles',JSON.stringify(data));
}

function selectProfile()
{
    document.getElementById('profile_selection').style.display='block';
    document.getElementById('profile_prepare').style.display='none';
    var data = getProfiles();
    var sel = document.getElementById('profile_select');
    for(var i=sel.length-1;i>=0;i--) {
        sel.options.remove(i);
    }
    for(var i=-1;i<data.length;i++) {
        var option = document.createElement("option");
        if(i==-1) {
            option.text = '--- select ---';
            option.value = '-1';
            option.selected = true;
        }
        else {
            option.text = data[i].name;
            option.value = '' + i;
        }
        sel.add(option);
    }
    loadCalibFrames();
}

function profileDeleteSelected()
{
    var data = getProfiles();
    var sel = document.getElementById('profile_select');
    try {
        var id = parseInt(sel.value);
        if(id < 0 || id>=data.length)
            return;
        data.splice(id,1);
        saveProfiles(data);
        selectProfile();
    }
    catch(e) { showError(e); }
}

function profileApply()
{
    var data = getProfiles();
    var sel = document.getElementById('profile_select');
    try {
        var id = parseInt(sel.value);
        if(id < 0 || id >= data.length)
            return;
        var opts = data[id].camera_options;
        var darks = data[id].darks;
        if(darks!=null) {
            var dselect = document.getElementById('stack_darks');
            dselect.value = darks;
            saveCalibValue(dselect);
        }
        restCall('post','/api/camera/options',opts,
            (e)=>{ loadControls(); },
            (e)=>{ 
                showError(e);
                loadControls(); 
            }
        );
    }
    catch(e) {
        console.log(e);
        showError("Select Profile");
    }
}

function deleteProfOpt(id)
{
    for(var i=0;i<g_profile_cam_opts.length;i++) {
        if(g_profile_cam_opts[i].id == id) {
            g_profile_cam_opts.splice(i,1);
            break;
        }
    }
    var el = document.getElementById('prof_opt_' + id);
    if(el)
        el.parentNode.removeChild(el);
}

function newProfileOptions(ctls)
{
    g_profile_cam_opts = [];
    var res = '';
    var has_ae = false;
    var has_awb = false;
    for(var i=0;i<ctls.length;i++) {
        var ctl = ctls[i];
        if(ctl.read_only)
            continue;
        if(ctl.option_id == 'auto_wb' && ctl.cur == 1)
            has_awb = true;
        if(ctl.option_id == 'auto_exp' && ctl.cur == 1)
            has_ae = true;
        if((ctl.option_id == 'exp' || ctl.option_id == 'gain') && has_ae)
            continue;
        if((ctl.option_id == 'wb' || ctl.option_id == 'wb_r' || ctl.option_id == 'wb_b') && has_awb)
            continue;
        var str = `<li id="prof_opt_${ctl.option_id}">${ctl.name}: ${ctl.cur} <button onclick="deleteProfOpt('${ctl.option_id}');">del</button></li>\n`;
        res = res + str;
        g_profile_cam_opts.push({"id":ctl.option_id, "value": ctl.cur});
    }
    document.getElementById('profile_items').innerHTML = res;
}

function showNotification(message,time=3.0,countdown=false,func=null)
{
    var el = document.getElementById('short_message');
    if(time <= 0.001) {
        el.innerHTML = '';
        if(func!=null) {
            func();
        }
        return;
    }
    if(time > 0 && message != null) {
        var msg = message;
        if(countdown)
            msg += ' in ' + time.toFixed(1) + 's';
        el.innerHTML = msg;
    }
    if(countdown) {
        setTimeout(()=> {
            showNotification(message,time - 0.2,true,func)
        },200);
    }
    else {
        setTimeout(() =>  {
            showNotification(null,0,false,func);
        },time * 1000);
    }
}

function profileNew()
{
    document.getElementById('profile_selection').style.display='none';
    document.getElementById('profile_prepare').style.display='block';
    document.getElementById('new_profile_name').value = '';
    restCall('get','/data/calibration/index.json',null,(data)=>{
        updateCalibFramesById(data,'profile_darks');
    });
    restCall('get','/api/camera/options',null,newProfileOptions);
}

function profileSave()
{
    var data = getProfiles();
    var name = document.getElementById('new_profile_name').value.trim();
    if(name == '') {
        showError('Profile Name is Empty');
        return;
    }

    var prof = {
        "name" : name,
        "camera_options" : g_profile_cam_opts,
        "darks" : document.getElementById('profile_darks').value
    };
    if(prof.darks == 'N/A' || prof.darks == '')
        prof.darks = null;
    for(var i=0;i<data.length;i++) {
        if(data[i].name == name) {
            data[i] = prof;
            prof = null;
            break;
        }
    }
    if(prof)
        data.push(prof);
    saveProfiles(data);
    selectProfile();
}

function isSolarSystemObject(p)
{
    return ['MOON','MERCURY','VENUS','MARS','JUPITER','SATURN','NEPTUNE','URANUS','SUN'].indexOf(p) != -1;
}

function getSolarSystemObject(p,lat,lon)
{
    p = p.trim();
    p = p.substring(0,1).toUpperCase() + p.substring(1).toLowerCase();
    var r2d = 180 / Math.PI;
    var d2r = Math.PI / 180;
    var rLat = lat * d2r;
    var rLon = lon * d2r;
    var date = new Date();
    var jd = JulianDate.gregorianDateToJulianDate(date.getUTCFullYear(),date.getUTCMonth()+1,date.getUTCDate(),date.getUTCHours(),date.getUTCMinutes(),date.getUTCSeconds());
    var body = CPReduce.bodies.indexOf(p);
    var res =  CPReduce.reduce(body,jd,[rLat,rLon,0]);
    var RA = res[0];
    var DEC = res[1];

	return [ formatRA(RA * r2d / 15), formatDEC(DEC *r2d) ]
}




window.onload = (event) => {
  run();
};

window.onresize = onResize;


