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


     
    $scope.$on('rosStateChanged', function() {
        // console.log('nodes actualizado desde home');
        // $scope.serverConnected = Ros.isServerConnnected();
        // console.log('El servidor esta: '+Ros.serverConnected());
        // $scope.serverState = Ros.getServerState();
        //}
      // console.log($scope.serverState);
    });   

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

    $scope.startNodes = function (nodes) {
        // for (var i=0; i<nodes.length-1; i++) {
            // console.log(nodes[i]);
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
        // }
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

  });
