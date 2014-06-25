'use strict';

angular.module('finderApp')
  .controller('RosTestCtrl', function ($scope, $rootScope, Auth, $location, Ros) {
    // Connecting to ROS
    // -----------------
    // var ros = new ROSLIB.Ros();
    // var ros = $rootScope.ros;
    // console.log($rootScope.datas);
    var ros = Ros.ros;
    Ros.connect(9090);
    // console.log(Ros.message);

    // console.log(Ros.message);

    // If there is an error on the backend, an 'error' emit will be emitted.
    ros.on('error', function(error) {
      console.log(error);
    });

    // Find out exactly when we made a connection.
    ros.on('connection', function() {
      console.log('Connection made!');
    });

    ros.getTopics( function (topics) {
      console.log(topics[0]);
    });

    ros.getNodes( function (nodes) {
      console.log(nodes);
    });

    // Create a connection to the rosbridge WebSocket server.
    // ros.connect('ws://localhost:9090');

    // Publishing a Topic
    // ------------------

    // First, we create a Topic object with details of the topic's name and message type.
    var cmdVel = new ROSLIB.Topic({
      ros : ros,
      name : '/cmd_vel',
      messageType : 'geometry_msgs/Twist'
    });

    // Then we create the payload to be published. The object we pass in to ros.Message matches the 
    // fields defined in the geometry_msgs/Twist.msg definition.
    var twist = new ROSLIB.Message({
      linear : {
        x : 0.1,
        y : 0.2,
        z : 0.3
      },
      angular : {
        x : -0.1,
        y : -0.2,
        z : -0.3
      }
    });

    // And finally, publish.
    // cmdVel.publish(twist);

    //Subscribing to a Topic
    //----------------------

    // Like when publishing a topic, we first create a Topic object with details of the topic's name 
    // and message type. Note that we can call publish or subscribe on the same topic object.
    // var listener = new ROSLIB.Topic({
    //   ros : ros,
    //   name : '/listener',
    //   messageType : 'std_msgs/String'
    // });

    // var listenerAlive = new ROSLIB.Topic({
    //   ros : ros,
    //   name : '/alive',
    //   messageType : 'std_msgs/Int32'
    // });

    // Then we add a callback to be called every time a message is published on this topic.
    // listener.subscribe(function(message) {
    //   console.log('Received message on ' + listener.name + ': ' + message.data);

    //   // If desired, we can unsubscribe from the topic as well.
    //   listener.unsubscribe();
    // });

    // listenerAlive.subscribe(function (message) {
    //   //console.log('Se recibió un mensaje de alive con el dato ' + listenerAlive.name + " " + message.data);


    // });


    // setInterval(function () {
    //   ros.getNodes( function (nodes) {
    //     console.log(nodes);
    //   })}, 1000);
    // Calling a service
    // -----------------

    // First, we create a Service client with details of the service's name and service type.
    // var addTwoIntsClient = new ROSLIB.Service({
    //   ros : ros,
    //   name : '/add_two_ints',
    //   serviceType : 'rospy_tutorials/AddTwoInts'
    // });

    // Then we create a Service Request. The object we pass in to ROSLIB.ServiceRequest matches the 
    // fields defined in the rospy_tutorials AddTwoInts.srv file.
    // var request = new ROSLIB.ServiceRequest({
    //   a : 1,
    //   b : 2
    // });

    // Finally, we call the /add_two_ints service and get back the results in the callback. The result
    // is a ROSLIB.ServiceResponse object.
    // addTwoIntsClient.callService(request, function(result) {
    //   console.log('Result for service call on ' + addTwoIntsClient.name + ': ' + result.sum);
    // });

    // Setting a param value
    // ---------------------

    // ros.getParams(function(params) {
    //   console.log(params);
    // });

    // First, we create a Param object with the name of the param.
    // var maxVelX = new ROSLIB.Param({
    //   ros : ros,
    //   name : 'max_vel_y'
    // });

    //Then we set the value of the param, which is sent to the ROS Parameter Server.
    // maxVelX.set(0.8);
    // maxVelX.get(function(value) {
    //   console.log('MAX VAL: ' + value);
    // });

    // Getting a param value
    // ---------------------

    // var favoriteColor = new ROSLIB.Param({
    //   ros : ros,
    //   name : 'favorite_color'
    // });

    // favoriteColor.set('red');
    // favoriteColor.get(function(value) {
    //   console.log('My robot\'s favorite color is ' + value);
    // });


    function init() {
      // Create the main viewer.
      var viewer = new MJPEGCANVAS.Viewer({
        divID : 'mjpeg',
        host : 'localhost',
        width : 640,
        height : 480,
        // image_transport : "compressed",
        topic : '/usb_cam_1'
      });
    }

    // init();
  });