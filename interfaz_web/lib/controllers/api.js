'use strict';

var mongoose = require('mongoose'),
	sys = require('sys');

var nodes = {
  roscore : {
    start: ['roscore',[]],
    stop: "kill -9 $(ps aux | grep roscore | grep -v grep | awk '{print $2}'); kill -9 $(ps aux | grep rosmaster | grep -v grep | awk '{print $2}'); kill -9 $(ps aux | grep rosout | grep -v grep | awk '{print $2}')",
    evalState: "ps aux | grep -v grep | egrep 'roscore' -c",
    state: "0"
  },
  rosbridge : {
    start: ['rosrun',['rosbridge_server','rosbridge_websocket']],
    stop: "kill -9 $(ps aux | grep rosbridge | grep -v grep | awk '{print $2}')",
    evalState: "ps aux | grep -v grep | egrep 'rosbridge' -c",
    state: "0"
  },
  rosalive : {
    start: ['rostopic', ['pub','/alive','std_msgs/Int16','1','-r','1','__name:=rosalive']],
    stop: "kill -9 $(ps aux | grep alive | grep -v grep | awk '{print $2}')",
    evalState: "ps aux | grep -v grep | egrep 'alive' -c",
    state: "0"
  },
  rostractionarduino : {
    start: ['rosrun',['rosserial_python','serial_node.py','/dev/traction_arduino','__name:=traction_arduino']],
    stop: "kill -9 $(ps aux | grep traction_arduino | grep -v grep | awk '{print $2}')",
    evalState: "ps aux | grep -v grep | egrep 'traction_arduino' -c",
    state: "0"
  },
  rossensorarduino : {
    start: ['rosrun',['rosserial_python','serial_node.py','/dev/sensor_arduino','__name:=sensor_arduino']],
    stop: "kill -9 $(ps aux | grep sensor_arduino | grep -v grep | awk '{print $2}')",
    evalState: "ps aux | grep -v grep | egrep 'sensor_arduino' -c",
    state: "0"
  },
  rostalonarduino : {
    start: ['rosrun',['rosserial_python','serial_node.py','/dev/talon_arduino','__name:=talon_arduino']],
    stop: "kill -9 $(ps aux | grep talon_arduino | grep -v grep | awk '{print $2}')",
    evalState: "ps aux | grep -v grep | egrep 'talon_arduino' -c",
    state: "0"
  },
  rosarmarduino : {
    start: ['rosrun',['rosserial_python','serial_node.py','/dev/arm_arduino','__name:=arm_arduino']],
    stop: "kill -9 $(ps aux | grep arm_arduino | grep -v grep | awk '{print $2}')",
    evalState: "ps aux | grep -v grep | egrep 'arm_arduino' -c",
    state: "0"
  },
  rosirarduino : {
    start: ['rosrun',['rosserial_python','serial_node.py','/dev/ir_arduino','__name:=ir_arduino']],
    stop: "kill -9 $(ps aux | grep ir_arduino | grep -v grep | awk '{print $2}')",
    evalState: "ps aux | grep -v grep | egrep 'ir_arduino' -c",
    state: "0"
  },
  rosbasenode : {
    start: ['rosrun',['finder','base_node.py']],
    stop: "kill -9 $(ps aux | grep base_node.py | grep -v grep | awk '{print $2}')",
    evalState: "ps aux | grep -v grep | egrep 'base_node.py' -c",
    state: "0"
  },
  rosarmnode : {
    start: ['rosrun',['finder','arm_node.py']],
    stop: "kill -9 $(ps aux | grep '/arm_node.py' | grep -v grep | awk '{print $2}')",
    evalState: "ps aux | grep -v grep | egrep '/arm_node.py' -c",
    state: "0"
  },
  rosforearmnode : {
    start: ['rosrun',['finder','forearm_node.py']],
    stop: "kill -9 $(ps aux | grep forearm_node.py | grep -v grep | awk '{print $2}')",
    evalState: "ps aux | grep -v grep | egrep 'forearm_node.py' -c",
    state: "0"
  },
  roswristnode : {
    start: ['rosrun',['finder','wrist_node.py']],
    stop: "kill -9 $(ps aux | grep wrist_node.py | grep -v grep | awk '{print $2}')",
    evalState: "ps aux | grep -v grep | egrep 'wrist_node.py' -c",
    state: "0"
  },
  rospalmnode : {
    start: ['rosrun',['finder','palm_node.py']],
    stop: "kill -9 $(ps aux | grep palm_node.py | grep -v grep | awk '{print $2}')",
    evalState: "ps aux | grep -v grep | egrep 'palm_node.py' -c",
    state: "0"
  },
  rosgrippernode : {
    start: ['rosrun',['finder','gripper_node.py']],
    stop: "kill -9 $(ps aux | grep gripper_node.py | grep -v grep | awk '{print $2}')",
    evalState: "ps aux | grep -v grep | egrep 'gripper_node.py' -c",
    state: "0"
  },
  rosleftnode : {
    start: ['rosrun',['finder','left_node.py']],
    stop: "kill -9 $(ps aux | grep left_node.py | grep -v grep | awk '{print $2}')",
    evalState: "ps aux | grep -v grep | egrep 'left_node.py' -c",
    state: "0"
  },
  rosrightnode : {
    start: ['rosrun',['finder','right_node.py']],
    stop: "kill -9 $(ps aux | grep right_node.py | grep -v grep | awk '{print $2}')",
    evalState: "ps aux | grep -v grep | egrep 'right_node.py' -c",
    state: "0"
  },
  rosfrnode : {
    start: ['rosrun',['finder','fr_node.py']],
    stop: "kill -9 $(ps aux | grep fr_node.py | grep -v grep | awk '{print $2}')",
    evalState: "ps aux | grep -v grep | egrep 'fr_node.py' -c",
    state: "0"
  },
  rosflnode : {
    start: ['rosrun',['finder','fl_node.py']],
    stop: "kill -9 $(ps aux | grep fl_node.py | grep -v grep | awk '{print $2}')",
    evalState: "ps aux | grep -v grep | egrep 'fl_node.py' -c",
    state: "0"
  },
  rosbrnode : {
    start: ['rosrun',['finder','br_node.py']],
    stop: "kill -9 $(ps aux | grep br_node.py | grep -v grep | awk '{print $2}')",
    evalState: "ps aux | grep -v grep | egrep 'br_node.py' -c",
    state: "0"
  },
  rosblnode : {
    start: ['rosrun',['finder','bl_node.py']],
    stop: "kill -9 $(ps aux | grep bl_node.py | grep -v grep | awk '{print $2}')",
    evalState: "ps aux | grep -v grep | egrep 'bl_node.py' -c",
    state: "0"
  },
  roscam : {
    start: ['roslaunch',['finder','cam_launch_test.launch']],
    stop: "kill -9 $(ps aux | grep '/usb_cam_node' | grep -v grep | awk '{print $2}')",
    evalState: "ps aux | grep -v grep | egrep '/usb_cam_node' -c",
    state: "0"
  },
  rosmjpegserver : {
    start: ['rosrun',['mjpeg_server','mjpeg_server']],
    stop: "kill -9 $(ps aux | grep mjpeg_server | grep -v grep | awk '{print $2}')",
    evalState: "ps aux | grep -v grep | egrep 'mjpeg_server' -c",
    state: "0"
  },
  rosbatterymonitor : {
    start: ['rosrun',['finder','battery_monitor.py']],
    stop: "kill -9 $(ps aux | grep battery_monitor | grep -v grep | awk '{print $2}')",
    evalState: "ps aux | grep -v grep | egrep 'battery_monitor' -c",
    state: "0"
  },
  rosapi : {
    start: ['rosrun',['rosapi','rosapi']],
    stop: "kill -9 $(ps aux | grep rosapi | grep -v grep | awk '{print $2}')",
    evalState: "ps aux | grep -v grep | egrep 'rosapi' -c",
    state: "0"
  },
  roscontrol : {
    start: ['rosrun',['finder','control_interface.py']],
    stop: "kill -9 $(ps aux | grep control_interface.py | grep -v grep | awk '{print $2}')",
    evalState: "ps aux | grep -v grep | egrep 'control_interface.py' -c",
    state: "0"
  },
  rosirnode : {
    start: ['rosrun',['finder','ir_node.py']],
    stop: "kill -9 $(ps aux | grep ir_node.py | grep -v grep | awk '{print $2}')",
    evalState: "ps aux | grep -v grep | egrep 'ir_node.py' -c",
    state: "0"
  },
  roslasernode : {
    start: ['roslaunch',['finder','test_slam.launch']],
    stop: "kill -9 $(ps aux | grep '/laser_node' | grep -v grep | awk '{print $2}')",
    evalState: "ps aux | grep -v grep | egrep '/laser_node' -c",
    state: "0"
  }
};

var evalState = function (node) {
	var exec = require('child_process').exec;
	var proc = exec(nodes[node].evalState, function (error, stdout, stderr) {
                if (node === 'roslasernode') {
                console.log(node);
                console.log(stdout[0]);      }              
                if (stdout[0] !== '0') {
		    nodes[node].state = '1';
                }
                else {
                    nodes[node].state = '0';
                }
	});
};

setInterval( function () {
	for(var node in nodes) {
		evalState(node);
	}
}, 500);

exports.laptopBattery = function(req, res) {

	var exec = require('child_process').exec;

	function my_exec(command, callback) {
		var proc = exec(command);

		var list = [];
		proc.stdout.setEncoding('utf8');

		proc.stdout.on('data', function (chunk) {
			list.push(chunk);
		});

		proc.stdout.on('end', function () {
			callback(list.join());
		});
	}

	my_exec('acpi', function (stdout) {
		// var content stdout;
		var pattPercentage = /[0-9]*\x25/;
		var pattState = /[DC].*harging/;
		var pattTime = /[0-9][0-9]:[0-9]*/;
		// var result = stdout.match(pattPercentage);
		return res.json({'Percentage': stdout.match(pattPercentage)[0],
						 'State': stdout.match(pattState)[0],
						 'Time': stdout.match(pattTime)[0]});
	});
};

exports.startNode = function (req, res) {
	var node_name = req.body.node;

	var spawn = require('child_process').spawn,
      node = spawn(nodes[node_name].start[0],nodes[node_name].start[1]);
	node.name = node_name;

	node.stdout.on('data', function (data) {
		console.log("Node: " + node.name);
	  console.log('stdout: ' + data);
	});

	node.stderr.on('data', function (data) {
		console.log("Node: " + node.name);
	  console.log('stderr: ' + data);
	});

	node.on('exit', function (code) {
		console.log("Node: " + node.name);
    console.log('child process exited with code ' + code);
	});

	return res.json(true);
};

exports.stopNode = function (req, res) {
	var node_name = req.body.node;

	var exec = require('child_process').exec,
		node = exec(nodes[node_name].stop, function (error, stdout, stderr) {
	});

	node.stdout.on('data', function (data) {
//		console.log("Node: " + node);
//	  console.log('stdout: ' + data);
	});

	node.stderr.on('data', function (data) {
//		console.log("Node: " + node);
//	  console.log('stderr: ' + data);
	});

	node.on('exit', function (code) {
//		console.log("Node: " + node);
//	    console.log('child process exited with code ' + code);
	});

	return res.json(true);
};

exports.getNodes = function(req, res) {
	var node_name = {};
	for(var node in nodes) {
		node_name[node] = nodes[node].state;
	}
	return res.json(node_name);
};
