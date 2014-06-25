'use strict';

angular.module('finderApp')
  .controller('HomeCtrl', function ($scope, $interval, $rootScope, $http, Auth, $location, Ros) {
    
    $scope.nodes = Ros.node.getNodes();
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

    $scope.$on('nodesUpdated', function() {
        // console.log('nodes actualizado desde home');
        $scope.nodes = Ros.node.getNodes();
      });

  });