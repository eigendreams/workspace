'use strict';

angular.module('finderApp')
  .controller('NavbarCtrl', function ($scope, $location, $interval, $http, Auth, Ros) {
    
    
    // console.log(Ros.isConnected());

    $scope.rosState = 'Disconnected';
    $scope.isConnected = false;
    $scope.isCommunicated = false;
    $scope.laptopBattery = 0;
    $scope.robotBattery = 0;
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

    $scope.getRosState = Ros.getRosState;

    $scope.$watch('getRosState()', function(newVal) {
      // console.log("New Data", newVal);
      $scope.rosState = newVal;
    });

    $scope.$on('nodesUpdated', function() {
      // console.log('nodes actualizado desde home');
      // $scope.serverConnected = Ros.isServerConnnected();
      // console.log('El servidor esta: '+Ros.serverConnected());
      $scope.serverState = Ros.getServerState();
      // console.log($scope.serverState);
    });

    $scope.rosDisconnect = function () { Ros.disconnect(); };
    $scope.rosConnect = function () { Ros.connect(); };

    $interval(function () {

        if ($scope.rosState === 'Connected') {

          $scope.laptopBattery = Ros.topic.getData('batteryLevel');
          $scope.robotBattery = Ros.topic.getData('volt');
        }

    }, 3000);

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
