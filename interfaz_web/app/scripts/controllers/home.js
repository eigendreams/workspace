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
        for (var topic in $scope.topics) {
            // console.log(topic);
            Ros.topic.updateData(topic);
            if (topic == 'baseDes') {
                $scope.topics[topic] = Ros.topic.getData(topic).toFixed(2);
            } else {
                $scope.topics[topic] = Ros.topic.getData(topic);
                // console.log($scope.topics[topic]);
            }
            
        }
        // }
        
    }, 300);

  });
