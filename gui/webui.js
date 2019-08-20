function cb_process(){
    var cb_stereo_left = document.getElementById("cb-left-stereo");
    var cb_stereo_right = document.getElementById("cb-right-stereo");

    if (cb_stereo_left.checked == true){
        // Populate video source 
        video_left.src = "http://" + robot_IP + ":8080/stream?topic=/rs2_ros/stereo/left/image_rect_raw&type=ros_compressed";
        // video_left.src = "http://" + robot_IP + ":8080/stream?topic=/left/debug_left&type=mjpeg&quality=20";
    }else{
        video_left.src ="";
    }

    if (cb_stereo_right.checked == true){
        // Populate video source 
        video_right.src = "http://" + robot_IP + ":8080/stream?topic=/rs2_ros/stereo/right/image_rect_raw&type=ros_compressed";
    }else{
        video_right.src ="";
    }
}

window.onload = function () {

    snapshot_state = 0; // state 0 = waiting for dark images, state 1 = waiting for bright images
    robot_IP = location.hostname;
    document.getElementById("server-ip").innerHTML = 'Server IP:' + window.location.hostname;


    ros = new ROSLIB.Ros({
        url: "ws://" + robot_IP + ":9090"
    });
    

    server_status = document.getElementById("roslibjs-status");
    ros.on('connection', function() {
        server_status.innerHTML = "Socket OK";
        server_status.style.color = "limegreen";
    });

    ros.on('error', function(error) {
        server_status.innerHTML = "ERROR: " + error;
        server_status.style.color = "crimson";
    });

    ros.on('close', function() {
        server_status.innerHTML = "Socket Closed";
        server_status.style.color = "crimson";
    });

    //// CLEAR CONSOLE ROUTINE
    document.getElementById("btn-clear").onclick = function(){
        var myConsole = document.getElementById("console-display");
        myConsole.innerHTML = "";
    }

    //// Preview for exposure and gain settings

    var exposure_param = new ROSLIB.Param({
        ros : ros,
        name : '/rs2_ros/stereo/exposure'
    });

    var gain_param = new ROSLIB.Param({
        ros : ros,
        name : '/rs2_ros/stereo/gain'
    });

    // for the dark preview button
    preview_btn_dark = document.getElementById("btn-dark-exposure-preview");
    var dark_exposure = document.getElementById("text-dark-exposure");
    var dark_gain = document.getElementById("text-dark-gain");
    preview_btn_dark.onclick = function(){
        dark_exposure.style.backgroundColor = "";
        dark_gain.style.backgroundColor = "";
        exposure_param.set(Number(dark_exposure.value));
        gain_param.set(Number(dark_gain.value));

        exposure_param.get(function(value){
            if(value == dark_exposure.value)
                dark_exposure.style.backgroundColor = "lightgreen";
            else
                dark_exposure.style.backgroundColor = "red";
        });

        gain_param.get(function(value){
            if(value == dark_gain.value)
                dark_gain.style.backgroundColor = "lightgreen";
            else
                dark_gain.style.backgroundColor = "red";
        });
    }

    // for the bright preview button
    preview_btn_bright = document.getElementById("btn-bright-exposure-preview");
    var bright_exposure = document.getElementById("text-bright-exposure");
    var bright_gain = document.getElementById("text-bright-gain");
    preview_btn_bright.onclick = function(){
        bright_exposure.style.backgroundColor = "";
        bright_gain.style.backgroundColor = "";
        exposure_param.set(Number(bright_exposure.value));
        gain_param.set(Number(bright_gain.value));

        exposure_param.get(function(value){
            if(value == bright_exposure.value)
            bright_exposure.style.backgroundColor = "lightgreen";
            else
            bright_exposure.style.backgroundColor = "red";
        });

        gain_param.get(function(value){
            if(value == bright_gain.value)
                bright_gain.style.backgroundColor = "lightgreen";
            else
                bright_gain.style.backgroundColor = "red";
        });
    }

    //// Recording button

    var takenow_service = new ROSLIB.Service({
        ros : ros,
        name : '/take_now',
        serviceType : 'std_srv/SetBool'
    });

    var take_now_request_dark = new ROSLIB.ServiceRequest({
        data : true
    });

    var take_now_request_bright = new ROSLIB.ServiceRequest({
        data : false
    });

    var frame_status = document.getElementById("record-frame-status");
    var image_dark_left = document.getElementById("snapshot-dark-left");
    var image_dark_right = document.getElementById("snapshot-dark-right");
    var image_bright_left = document.getElementById("snapshot-bright-left");
    var image_bright_right = document.getElementById("snapshot-bright-right");


    document.getElementById("btn-record-frame").onclick = function(){
        image_dark_left.src = ""
        image_dark_right.src = ""
        image_bright_left.src = ""
        image_bright_right.src = ""
        snapshot_state = 0;
        frame_status.innerHTML = "Taking Dark Image...";
        exposure_param.set(Number(dark_exposure.value));
        gain_param.set(Number(dark_gain.value));
        image_dark_left.src = "http://" + robot_IP + ":8080/snapshot?topic=/snapshot_dark_left"
        image_dark_right.src = "http://" + robot_IP + ":8080/snapshot?topic=/snapshot_dark_right"
                takenow_service.callService(take_now_request_dark, function(result){
            if (result.success == true)
            {
                snapshot_state = 1;
                frame_status.innerHTML = "Taking Bright Image...";
                exposure_param.set(Number(bright_exposure.value));
                gain_param.set(Number(bright_gain.value));
                image_bright_left.src = "http://" + robot_IP + ":8080/snapshot?topic=/snapshot_bright_left"
                image_bright_right.src = "http://" + robot_IP + ":8080/snapshot?topic=/snapshot_bright_right"
                takenow_service.callService(take_now_request_bright,function(result){
                    if (result.success == true)
                    {
                        frame_status.innerHTML = "Success!";
                    } 
                    else
                        frame_status.innerHTML = "Error in bright image taking";
                });
            }else   
                frame_status.innerHTML = "Error in dark image taking";

            
        });
    };


    // get handle for video placeholder
    video_left = document.getElementById('video-left');
    video_right = document.getElementById('video-right');

    subscribeRosout();
    subscribePoseInfo();
    subscribeCameraInfo();

    // subscribeSnapshot();

    cb_process();

}
