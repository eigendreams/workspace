'use strict';

angular.module('finderApp')
  .controller('HomeCtrl', function ($scope, $interval, $timeout, $rootScope, $http, Auth, $location, Ros) {
    
    $scope.nodes = Ros.node.getNodes();
    $scope.serverIP = Ros.getServerIP();
    $scope.videoQuality = 30;
    $scope.videoReset = true;
    $scope.topics = Ros.topic.getNames();
    $scope.rosState = Ros.getRosState();
    $scope.rosDisconnect = Ros.disconnect;
    $scope.rosConnect = Ros.connect;
    $scope.termalsensorData = [[1,50,100,250],[1,50,100,47],[14,27,67,146],[113,232,43,49]];
    $scope.co2sensorData = 0;
    $scope.video1 = 1;
    $scope.video2 = 1;
    $scope.video3 = 1;
    $scope.video4 = 1;
    $scope.ledsValue = 0;

    $scope.listenerGroup = {
        groupOne : {
            baseDes: 0, 
            leftOut: 0, 
            rightOut: 0,
            frOut: 0,
            flOut: 0,
            brOut: 0,
            blOut: 0,
            baseOut: 0
        },
        groupSensor : {
            co2: 0,
            pitch: 0,
            roll: 0,
            batteryLevel: 0
        },
        groupBase : {
            leftOut: 0, 
            rightOut: 0,
            frOut: 0,
            flOut: 0,
            brOut: 0,
            blOut: 0
        },
        groupArm : {
            baseOut: 0,
            baseDes: 0,
            armOut: 0,
            forearmOut: 0,
            wristOut: 0,
            palmOut: 0,
            gripperOut: 0
        }
    }

    $scope.sliderConfig = {
        min: 1,
        max: 90,
        step: 2
    }
    
    $scope.sliderLedsConfig = {
        min: 0,
        max: 255,
        step: 15
    }

    $scope.setPrice = function(price) {
        $scope.price = price;    
    }

    $scope.setActiveListener = function (selection) {
        for (var item in $scope.activeListener) {
            if (item == selection) {
                $scope.activeListener[item] = true;
            } else {
                $scope.activeListener[item] = false;
            }
        }
    };
    // $scope.serverConnected = Ros.serverConnected;


    $scope.toggleRos = function () {
        if ($scope.rosState === 'Disconnected') {
            Ros.connect();
        } else {
            Ros.disconnect();
        }
    };

    $scope.toggleNode = function (node) {
        if($scope.nodes[node] == '0') {
            Ros.node.start(node);
        }
        else {
            Ros.node.stop(node);
        }
    };

    $scope.startNodes = function (line) {
        console.log(line);
        switch (line) {
            case 1:
                Ros.node.start('roscore');
                $timeout( function () {
                    Ros.node.start('rosbridge');
                    $timeout( function () {
                        Ros.node.start('rosalive');
                        $timeout( function () {
                            Ros.connect();
                        }, 2000);
                    }, 2000);
                }, 4000);
                break;
            case 2:
                Ros.node.start('rostractionarduino');
                Ros.node.start('rostalonarduino');
                Ros.node.start('rosarmarduino');
                Ros.node.start('rossensorarduino');
                Ros.node.start('rosirarduino');
                break;
            case 3:
                Ros.node.start('rosleftnode');
                Ros.node.start('rosrightnode');
                Ros.node.start('rosbrnode');
                Ros.node.start('rosblnode');
                Ros.node.start('rosfrnode');
                Ros.node.start('rosflnode');
                break;
            case 4:
                Ros.node.start('rosbasenode');
                Ros.node.start('rosarmnode');
                Ros.node.start('rosforearmnode');
                Ros.node.start('roswristnode');
                Ros.node.start('rospalmnode');
                Ros.node.start('rosgrippernode');
                break;
            case 5:
                Ros.node.start('rosmjpegserver');
                Ros.node.start('roscam');
                break;
            case 6:
                Ros.node.start('roscontrol');
                break;
            case 7:
                Ros.node.start('rosleftnode');
                Ros.node.start('rosrightnode');
                Ros.node.start('rosbrnode');
                Ros.node.start('rosblnode');
                Ros.node.start('rosfrnode');
                Ros.node.start('rosflnode');
                break;
        }
    }

    $scope.toggleVideoQuality = function () {
        if ($scope.videoQuality==20) {
            $scope.videoQuality = 15;
        }
        else {
            $scope.videoQuality = 20;
        }
    };

    $scope.$on('nodesUpdated', function() {
        $scope.nodes = Ros.node.getNodes();
        // console.log($scope.nodes);
        // $scope.rosState = Ros.getRosState();
    });

    $scope.$on('rosStateChanged', function() {
        $scope.rosState = Ros.getRosState();
        console.log("Cambió ros " + $scope.rosState);
    });

    $interval(function () {

        if ($scope.rosState === 'Connected') {
            // console.log("Mostando el intervalo");

            for (var topic in $scope.listenerGroup['groupBase']) {
                $scope.listenerGroup['groupBase'][topic] = Ros.topic.getData(topic);
            }

            for (var topic in $scope.listenerGroup['groupArm']) {
                if (topic == 'baseDes') {
                    $scope.listenerGroup['groupArm'][topic] = Ros.topic.getData(topic).toFixed(2);
                } else {
                    $scope.listenerGroup['groupArm'][topic] = Ros.topic.getData(topic);
                }
                
            }

            $scope.termalsensorData = Ros.topic.getData('irOut');
            $scope.co2sensorData = Ros.topic.getData('co2');
        }

    }, 200);

    $scope.publishData = function (topic, data) {
        console.log("publishData");
        Ros.topic.publishData(topic, data);
    }

    $scope.$watch('ledsValue', function() {
        console.log('hey, myVar has changed! ' + $scope.ledsValue);
        Ros.topic.publishData('leds', $scope.ledsValue);
    });


    // $scope.viewer;
    $scope.mapscale = 1;
    $scope.gridClient;

    $scope.scaleMap = function (rate) {
        $scope.mapscale += rate;
        // $scope.viewer.scaleToDimensions($scope.gridClient.currentGrid.width * $scope.mapscale, $scope.gridClient.currentGrid.height * $scope.mapscale);
        // $scope.viewer.scaleToDimensions($scope.gridClient.currentGrid.width * 0.9, $scope.gridClient.currentGrid.height * 0.9);
        $scope.viewer.scaleToDimensions(10, 10);
    };

    $scope.initMap = function () {

        // Create the main viewer.
        $("#map").empty();
        $scope.viewer = new ROS2D.Viewer({
            divID : 'map',
            width : 600,
            height : 300
        });
        

        // Setup the map client.
        $scope.gridClient = new OccupancyGridSrvClient({
            ros : Ros.ros,
            rootObject : $scope.viewer.scene,
            service: '/map'
        });

        // var gridClient = new ROS2D.OccupancyGridClient({
        //     ros : Ros.ros,
        //     rootObject : $scope.viewer.scene,
        //     topic: '/map',
        //     continuous: true
        // });

        // $scope.viewer.scaleToDimensions(gridClient.currentGrid.width, gridClient.currentGrid.height);

        // var grid = new ROS2D.OccupancyGrid;
        // Scale the canvas to fit to the map
        $scope.gridClient.on('change', function(){
            console.log("El mapa cambió");
            // $scope.viewer.scaleToDimensions($scope.gridClient.currentGrid.width, $scope.gridClient.currentGrid.height);
            // console.log(gridClient.currentGrid.width);
            // console.log(gridClient.currentGrid.height);
            // // $scope.viewer.shift(30,30);
            // $scope.viewer.shift(-1 * $scope.gridClient.currentGrid.width / 2 , -1 * $scope.gridClient.currentGrid.height / 2);
        });

        // $scope.viewer.shift(-1 * $scope.gridClient.currentGrid.width / 2 , -1 * $scope.gridClient.currentGrid.height / 2);
    };

    var OccupancyGridSrvClient = function(options) {
        var that = this;
        options = options || {};
        var ros = options.ros;
        var service = options.service || '/static_map';
        this.rootObject = options.rootObject || new createjs.Container();

        // current grid that is displayed
        this.currentGrid = null;

        // Setting up to the service
        var rosService = new ROSLIB.Service({
            ros : ros,
            name : service,
            serviceType : 'nav_msgs/GetMap',
            compression : 'png'
        });


        $interval(function () {
            console.log("Se inicio interval para el servicio");
            rosService.callService(new ROSLIB.ServiceRequest(),function(response) {
                // check for an old map
                if (that.currentGrid) {
                    that.rootObject.removeChild(that.currentGrid);
                }

                that.currentGrid = new ROS2D.OccupancyGrid({
                    message : response.map
                });
                that.rootObject.addChild(that.currentGrid);

                that.emit('change', that.currentGrid);
            });
        }, 2000);

    };

    OccupancyGridSrvClient.prototype.__proto__ = EventEmitter2.prototype;



  });
