'use strict';

angular.module('finderApp')
  .controller('HomeCtrl', function ($scope, $interval, $rootScope, $http, Auth, $location, Ros) {
    
    $scope.nodes = Ros.node.getNodes();
    $scope.serverIP = Ros.getServerIP();
    $scope.videoQuality = 30;
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
        if ($scope.videoQuality==30) {
            $scope.videoQuality = 50;
        }
        else {
            $scope.videoQuality = 30;
        }
    };

    $scope.$on('nodesUpdated', function() {
        // console.log('nodes actualizado desde home');
        $scope.nodes = Ros.node.getNodes();
      });

  });