'use strict';

angular.module('finderApp')
  .controller('HomeCtrl', function ($scope, $interval, $rootScope, $http, Auth, $location, Ros) {
    
    $scope.nodes = Ros.node.getNodes();
    $scope.serverIP = Ros.getServerIP();
    $scope.videoQuality = 30;
    $scope.topics = Ros.topic.getNames();
    // $scope.serverConnected = Ros.serverConnected;

    $scope.startNode = function (node) {
        Ros.node.start(node);
      };

    $scope.stopNode = function (node) {
        Ros.node.stop(node);
      };

    $scope.toggleNode = function (node) {
        // console.log($scope.nodes[node]);
        if($scope.nodes[node] == '0') {
            // console.log("Start Node");
            Ros.node.start(node);
        }
        else {
            // console.log("Stop node");
            Ros.node.stop(node);
        }
    };

    $scope.toggleVideoQuality = function () {
        if ($scope.videoQuality==80) {
            $scope.videoQuality = 15;
        }
        else {
            $scope.videoQuality = 80;
        }
    };

    $scope.$on('nodesUpdated', function() {
        // console.log('nodes actualizado desde home');
        $scope.nodes = Ros.node.getNodes();
        // Ros.topic.updateData('rightOut');
        // console.log(Ros.topic.getData('rightOut'));
        // $scope.topics.rightOut = Ros.topic.getData('rightOut');
        // console.log($scope.topics);
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
