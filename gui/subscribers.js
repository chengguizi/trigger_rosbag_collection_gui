function subscribeDriverUpdates(){
    var listener = new ROSLIB.Topic({
        ros : ros,
        name : '/mon_drivers/state',
        messageType : 'rosmon/State'
    })

    listener.subscribe(function(message){
        document.getElementById("state-msg-seq-driver").innerHTML = message.header.seq;

        for (i=0; i<message.nodes.length; i++){
            switch(message.nodes[i].name){
                case "imu":
                    ID = "state-msg-imu";
                    break;
                case "stereo":
                    ID = "state-msg-rs2";
                    break
                default:
                    console.log("unknown node" + message.nodes[i].name);
                    continue;
            }
            
            switch(message.nodes[i].state){
                case 0:
                    STATE = "Idle";
                    COLOR = "dimgray";
                    break;
                case 1:
                    STATE = "Running";
                    COLOR = "green";
                    break;
                case 2:
                    STATE = "Crashed";
                    COLOR = "crimson";
                    break;
                case 3:
                    STATE = "Waiting";
                    COLOR = "orange";
                    break;
            }
            document.getElementById(ID).innerHTML =  STATE + " (CPU " + ((message.nodes[i].user_load + message.nodes[i].system_load)*50).toFixed(1) + "%, Mem " + (message.nodes[i].memory / (1024*1024)).toFixed(1) + "MB)";
            document.getElementById(ID).style.color = COLOR;
        }
    });

}

function subscribeCameraInfo(){
    //// Subscribe to /rosout
    var listener = new ROSLIB.Topic({
        ros : ros,
        name : '/rs2_ros/stereo/camera_stats',
        messageType : 'rs2_ros/CameraStats'
    });

    listener.subscribe(function(msg){
        document.getElementById("text-exposure").innerHTML = msg.exposure;
        document.getElementById("text-gain").innerHTML = msg.gain;
        document.getElementById("text-seq").innerHTML = msg.header.seq;
        document.getElementById("text-lux").innerHTML = msg.meanLux;
    });

}


function subscribePoseInfo(){
    var listener = new ROSLIB.Topic({
        ros : ros,
        name : '/snapshot_pose',
        messageType : 'geometry_msgs/PoseStamped'
    });

    listener.subscribe(function(msg){
        document.getElementById("text-x").innerHTML = msg.pose.position.x.toFixed(1);
        document.getElementById("text-y").innerHTML = msg.pose.position.y.toFixed(1);
        document.getElementById("text-z").innerHTML = msg.pose.position.z.toFixed(1);
        
        var q = msg.pose.orientation;
        var yaw = - Math.atan2(2.0*(q.x*q.y + q.w*q.z), (q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z))/Math.PI*180;
        // var yaw = Math.asin(-2.0*(q.x*q.z - q.w*q.y))/Math.PI*180;
        // var yaw = Math.atan2(2.0*(q.y*q.z + q.w*q.x), q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z)/Math.PI*180;
        document.getElementById("text-yaw").innerHTML = yaw.toFixed(0);
        // atan2(2.0*(q.y*q.z + q.w*q.x), q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z);
    });
}

function subscribeRosout(){
    //// Subscribe to /rosout
    var listener = new ROSLIB.Topic({
        ros : ros,
        name : '/rosout_agg',
        messageType : 'rosgraph_msgs/Log'
    });
    
    // subscriber callback
    listener.subscribe(function(msg){
        // switch(msg.name){
        //     case '/imu':
        //         var myConsole = document.getElementById("console-display");
        //         var para = document.createElement("p");
        //         //var node = document.createTextNode("This is new.");
        //         //para.appendChild(node); 
        //         para.style.cssText = 'margin-bottom: 0px;';
        //         para.className = 'bg-warning';
        //         para.textContent = '> ' + msg.name + ": " + msg.msg;
        //         myConsole.appendChild(para);
        //         break;
        //     default : 
        //         console.log(msg.name + ": " + msg.msg);
        //         break;
        // }
        var CLASS_NAME;
        switch(msg.level){
            case 2: // info
                CLASS_NAME = "";
                break;
            case 4: // warning
                CLASS_NAME = "bg-warning";
                break;
            case 8: //error
            case 16: // fatal
                CLASS_NAME = "bg-danger";
                break;
        }
        var myConsole = document.getElementById("console-display");
        var para = document.createElement("p");
        para.textContent = 'seq ' + msg.header.seq + ' > ' +   msg.name + ": " + msg.msg;
        para.style.cssText = 'margin-bottom: 0px;';
        para.className = CLASS_NAME + " px-1";
        //myConsole.appendChild(para);
        myConsole.insertBefore(para, myConsole.childNodes[0] );

        if (msg.name == '/recorder' || msg.name ==  '/my_record_bag'){
            var myConsoleRecord = document.getElementById("console-record");

            myConsoleRecord.insertBefore(para, myConsoleRecord.childNodes[0] );
        }

        // console.log(msg);
    });
}