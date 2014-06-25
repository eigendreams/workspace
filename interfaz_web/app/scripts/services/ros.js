'use strict';

angular.module('finderApp')
  .factory('Ros', ['$rootScope', '$interval', '$timeout', '$http', function ($rootScope, $interval, $timeout, $http) {

    var ros = new ROSLIB.Ros();
    var connection_active = false;
    var communication_active = false;
    var nodeState;
    var nodes;
    


    var getNodes = function () {
      $http.get('/api/getNodes').
        success(function(data) {
          nodes = data;
        });

      $rootScope.$broadcast('nodesUpdated');
    };

    $interval( function () {
      getNodes();
    }, 1200);

    var promise;
    var laptopBattery = '0%';

    var setDisconnected = function () {
      communication_active = false;
    };

    var checkLaptopBattery = function () {
      // $http.get('/api/battery').
      //   success(function(data) {
      //     // $scope.post = data.post;
      //     // console.log(data);
      //     laptopBattery = data;
      //   });
    };

    var init = function () {

      var listenerAlive = new ROSLIB.Topic({
        ros : ros,
        name : '/alive',
        messageType : 'std_msgs/Int32'
      });

      listenerAlive.subscribe(function (message) {
        //console.log('Se recibió un mensaje de alive con el dato ' + listenerAlive.name + " " + message.data);
        // console.log("Connected");
        checkLaptopBattery();

        communication_active = true;
        $timeout.cancel(promise);
        promise = $timeout(setDisconnected, 3000);
        $rootScope.$apply();
        // console.log(laptopBattery);
      });
    };

    return {
      ros: ros,
      connect: function (port) {
        ros.close();
        ros.connect('ws://localhost:' + port);
        init();
        // connection_active = true;
        connection_active = true;
      },
      disconnect: function () {
        ros.close();
        // connection_active = true;
        connection_active = false;
      },
      isConnected: function () {
        return connection_active;
        // return false;
      },
      isCommunicated: function () {
        return communication_active;
        // return false;
      },
      getState: function () {
        var state = '';
        if(connection_active) {
          // console.log(communication_active);
          if (communication_active) {
            state = 'Connected';
          }
          else {
            state = 'Uncommunicated';
          }
        }
        else {
          state = 'Disconnected';
        }

        return state;
      },
      getLaptopBattery: function () {
        return laptopBattery;
      },
      node: {
        start : function (node) {
          $http.post('/api/startNode', {node: node}).
            success(function(data) {
          });
        },
        stop : function (node) {
          $http.post('/api/stopNode', {node: node}).
            success(function(data) {
            });
        },
        getNodes : function () {
          return nodes;
        }
      }
    };
  }]);
