function cb_process(){
    var cb_stereo_left = document.getElementById("cb-left-stereo");
    var cb_stereo_right = document.getElementById("cb-right-stereo");

    if (cb_stereo_left.checked == true){
        // Populate video source 
        video_left.src = "http://" + robot_IP + ":8080/stream?topic=/rs2_ros/" + camera_ns + "/left/image_rect_raw&type=ros_compressed";
        // video_left.src = "http://" + robot_IP + ":8080/stream?topic=/left/debug_left&type=mjpeg&quality=20";
    }else{
        video_left.src ="";
    }

    if (cb_stereo_right.checked == true){
        // Populate video source 
        video_right.src = "http://" + robot_IP + ":8080/stream?topic=/rs2_ros/" + camera_ns + "/right/image_rect_raw&type=ros_compressed";
    }else{
        video_right.src ="";
    }
}

window.onload = function () {

    var myURL = window.location.href
    var url = new URL(myURL)
    camera_ns = url.searchParams.get("ns"); // namespace
    // console.log(camera_ns);
    if (camera_ns == null){
        camera_ns = "stereo";
    }

    document.getElementById("text-camera-ns").innerHTML = camera_ns;

    snapshot_state = 0; // state 0 = waiting for dark images, state 1 = waiting for bright images
    robot_IP = location.hostname;
    document.getElementById("server-ip").innerHTML = 'Server IP:' + window.location.hostname;


    ros = new ROSLIB.Ros({
        url: "ws://" + robot_IP + ":9999"
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
        name : '/rs2_ros/" + camera_ns + "/exposure'
    });

    var gain_param = new ROSLIB.Param({
        ros : ros,
        name : '/rs2_ros/" + camera_ns + "/gain'
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
        image_dark_left.src = "";
        image_dark_right.src = "";
        image_bright_left.src = "";
        image_bright_right.src = "";

        document.getElementById("text-x").innerHTML="";
        document.getElementById("text-y").innerHTML="";
        document.getElementById("text-z").innerHTML="";
        document.getElementById("text-yaw").innerHTML="";
        document.getElementById("text-dx").innerHTML="";
        document.getElementById("text-dy").innerHTML="";
        document.getElementById("text-dz").innerHTML="";
        document.getElementById("text-dyaw").innerHTML="";

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
                        frame_status.innerHTML = "Error in bright image taking:" + result.message;
                });
            }else   
                frame_status.innerHTML = "Error in dark image taking:" + result.message;

            
        });
    };


    // get handle for video placeholder
    video_left = document.getElementById('video-left');
    video_right = document.getElementById('video-right');


    //// Routine for start / stop buttons for rosmon

    var reset_system_service = new ROSLIB.Service({
        ros : ros,
        name : '/trigger_rosbag_collection/start_stop',
        serviceType : 'rosmon/StartStop'
    });

    var request_restart_rs2 = new ROSLIB.ServiceRequest({
        node : 'rs2_ros',
        action : 3 // restart
    });

    var request_stop_rs2 = new ROSLIB.ServiceRequest({
        node : 'rs2_ros',
        action : 2 // stop
    });

    var request_restart_trigger = new ROSLIB.ServiceRequest({
        node : 'snapshot.py',
        action : 3 // restart
    });

    var request_stop_trigger = new ROSLIB.ServiceRequest({
        node : 'snapshot.py',
        action : 2 // stop
    });

    document.getElementById("btn-reset-rs2").onclick = function(){
        reset_system_service.callService(request_restart_rs2,function(result){});
    }

    document.getElementById("btn-stop-rs2").onclick = function(){
        reset_system_service.callService(request_stop_rs2,function(result){});
    }

    document.getElementById("btn-reset-trigger").onclick = function(){
        reset_system_service.callService(request_restart_trigger,function(result){});
    }

    document.getElementById("btn-stop-trigger").onclick = function(){
        reset_system_service.callService(request_stop_trigger,function(result){});
    }

    document.getElementById("btn-stop-record").onclick = function(){
        reset_system_service.callService(request_restart_trigger,function(result){});
    }

    subscribeRosout();
    subscribePoseInfo();
    subscribeCameraInfo();
    subscribeMonUpdates();

    // subscribeSnapshot();

    cb_process();

}
