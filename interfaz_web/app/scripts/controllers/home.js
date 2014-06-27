'use strict';

angular.module('finderApp')
  .controller('HomeCtrl', function ($scope, $interval, $timeout, $rootScope, $http, Auth, $location, Ros) {
    
    $scope.nodes = Ros.node.getNodes();
    $scope.serverIP = Ros.getServerIP();
    $scope.videoQuality = 30;
    $scope.topics = Ros.topic.getNames();
    $scope.rosState = Ros.getRosState();
    $scope.rosDisconnect = Ros.disconnect;
    $scope.rosConnect = Ros.connect;
        

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
            irOut: 0,
            co2: 0,
            pitch: 0,
            roll: 0
        }
    }

    $scope.activeListener = {
        groupSensor: false,
        groupOne: false
    };

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
        if ($scope.videoQuality==80) {
            $scope.videoQuality = 15;
        }
        else {
            $scope.videoQuality = 80;
        }
    };

    $scope.$on('nodesUpdated', function() {
        $scope.nodes = Ros.node.getNodes();
        // $scope.rosState = Ros.getRosState();
    });

    $scope.$on('rosStateChanged', function() {
        $scope.rosState = Ros.getRosState();
        console.log("Cambió ros " + $scope.rosState);
    });

    $interval(function () {
        // if(Ros.getState() === 'Connected') {
            // Ros.topic.updateData('rightOut');
            // Ros.topic.updateData('rightOut');
            // Ros.topic.updateData('rightOut');
        // console.log($scope.topics);

        for (var listenerGroup in $scope.activeListener) {
            if ($scope.activeListener[listenerGroup]) {
                for (var topic in $scope.listenerGroup[listenerGroup]) {
                    // console.log(topic);
                    // Ros.topic.updateData(topic);
                    if (topic == 'baseDes') {
                        $scope.listenerGroup[listenerGroup][topic] = Ros.topic.getData(topic).toFixed(2);
                    } else {
                        $scope.listenerGroup[listenerGroup][topic] = Ros.topic.getData(topic);
                        // console.log($scope.topics[topic]);
                    }
                    
                }
            }
        }

                
        // }
        
    }, 100);

  });
