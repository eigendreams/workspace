'use strict';

angular.module('finderApp')
  .controller('NavbarCtrl', function ($scope, $location, $interval, $http, Auth, Ros) {
    
    
    // console.log(Ros.isConnected());

    $scope.state = 'Disconnected';
    $scope.isConnected = false;
    $scope.isCommunicated = false;
    // $scope.isServerConnnected = Ros.isServerConnnected;
    // $scope.laptopBattery = '0%';
    // Ros.connect(9090);

    // $interval( function () {
    //   $scope.isConnected = Ros.isConnected();
    //   $scope.isCommunicated = Ros.isCommunicated();

    //   if($scope.isConnected){
    //     if($scope.isCommunicated){
    //       $scope.state = "Connected";
    //     }
    //     else{
    //       $scope.state = "Uncommunicated";
    //     }
    //   }
    //   else {
    //     $scope.state = "Disconnected";
    //   }
    // }, 1000);

    $scope.getState = Ros.getState;
    $scope.getLaptopBattery = Ros.getLaptopBattery;
    $scope.laptopBattery = '0%';
    $scope.serverConnected = true;

    $scope.$watch('getState()', function(newVal) {
      // console.log("New Data", newVal);
      $scope.state = newVal;
    });

    $scope.$watch('getLaptopBattery()', function(newVal) {
      // console.log("New Data", newVal);
      $scope.laptopBattery = newVal;
    });

    $scope.$on('nodesUpdated', function() {
        // console.log('nodes actualizado desde home');
        // $scope.serverConnected = Ros.isServerConnnected();
        // console.log('El servidor esta: '+Ros.serverConnected());
        $scope.serverConnected = Ros.serverConnected();
      });

    $scope.rosDisconnect = function () { Ros.disconnect(); };
    $scope.rosConnect = function () { Ros.connect(9090); };

    // $scope.checkLaptopBattery = function () { 
    //   $http.get('/api/battery').
    //     success(function(data) {
    //       // $scope.post = data.post;
    //       return data;
    //     });
    // };

    // $scope.$watch('checkLaptopBattery()', function(newVal) {
    //   // console.log("New Data", newVal);
    //   // $scope.state = newVal;
    //   console.log(newVal);
    // });

    $scope.menu = [{
      'title': 'Home',
      'link': '/home'
    }, {
      'title': 'Settings',
      'link': '/settings'
    }, {
      'title': 'Ros Test',
      'link': '/ros_test'
    }, {
      'title': 'Ros Nodes',
      'link': '/ros_nodes'
    }, {
      'title': 'Topics',
      'link': '/topics'
    }];
    
    $scope.logout = function() {
      Auth.logout()
      .then(function() {
        $location.path('/login');
      });
    };
    
    $scope.isActive = function(route) {
      return route === $location.path();
    };

  });
