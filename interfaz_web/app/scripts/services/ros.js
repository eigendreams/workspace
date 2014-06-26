'use strict';

angular.module('finderApp')
  .factory('Ros', ['$rootScope', '$interval', '$timeout', '$http', function ($rootScope, $interval, $timeout, $http) {

    // var serverIP = "192.168.88.253";
    var serverIP = "localhost";
    var ros = new ROSLIB.Ros();
    var rosConnectionActive = false;
    var rosCommunicationActive = false;
    var serverConnected = false;
    var serverConnectionPromise;
    var nodeState;
    var nodes;
    var outTopics = [
      '/left_out',
      '/right_out',
      '/fr_out',
      '/fl_out',
      '/br_out',
      '/bl_out',
      '/base_out',
      '/arm_out',
      '/forearm_out',
      '/wrist_out',
      '/palm_out',
      '/gripper_out'
    ];

    var topics = {};
    

    var getNodes = function () {
      $http.get('/api/getNodes').
        success(function(data) {
          nodes = data;
          // communication_active = true;
          serverConnected = true;
          $timeout.cancel(serverConnectionPromise);
          serverConnectionPromise = $timeout( function () {
            serverConnected = false;
          }, 3000);
          // $rootScope.$apply();
          // console.log('Server connected');
        })
        // error(function(data) {
        //   // console.log('Error en el server');
        //   serverConnected = false;
        // });

      $rootScope.$broadcast('nodesUpdated');
    };

    var getTopics = function () {
      if (rosCommunicationActive) {
        ros.getTopics( function (topics) {
          console.log(topics);
        });
      }
      else {
        console.log("Error: no hay coneccion de ros");
      }
    };

    $interval( function () {
      getNodes();
      // getTopics();
      // ros.getTopics( function (topics) {
      //     console.log(topics);
      //   });
      // console.log(serverConnected);
    }, 1200);

    var promise;
    var serverConnectionPromise;
    var laptopBattery = '0%';

    var setDisconnected = function () {
      rosCommunicationActive = false;
    };

    // var setServerDisconnected = function () {
    //   serverConnected = false;
    //   console.log('Server disconnected');
    // };

    var checkLaptopBattery = function () {
      // $http.get('/api/battery').
      //   success(function(data) {
      //     // $scope.post = data.post;
      //     // console.log(data);
      //     laptopBattery = data;
      //   });
    };

    // var createTopic = function (topic) {

    //   var listener = new ROSLIB.Topic({
    //     ros : ros,
    //     name : topic,
    //     messageType : 'std_msgs/Int32'
    //   });

    // }

    var init = function () {

      topics = {};

      for (var i=0; i<outTopics.length; i++) {
        var listener = new ROSLIB.Topic({
          ros : ros,
          name : outTopics[i],
          messageType : 'std_msgs/Int16'
        });

        topics[outTopics[i]] = listener;
      }

      console.log(topics);


      var listenerAlive = new ROSLIB.Topic({
        ros : ros,
        name : '/alive',
        messageType : 'std_msgs/Int16'
      });

      listenerAlive.subscribe(function (message) {
        //console.log('Se recibió un mensaje de alive con el dato ' + listenerAlive.name + " " + message.data);
        // console.log("Connected");
        // checkLaptopBattery();

        rosCommunicationActive = true;
        $timeout.cancel(promise);
        promise = $timeout(setDisconnected, 3000);
        // $rootScope.$apply();
        // console.log(laptopBattery);
      });

      console.log("A punto de pedir topicos");

      // ros.getTopics( function (topics) {
      //     console.log(topics);
      //   });
    };

    return {
      ros: ros,
      getServerIP: function () { 
        return serverIP; 
      },
      serverConnected: function () {
        return serverConnected;
      },
      topics : topics,
      connect: function (port) {
        console.log("Tratando de conectarse");
        ros.close();
        ros.connect('ws://' + serverIP + ':' + port);
        init();
        // connection_active = true;
        rosConnectionActive = true;
      },
      disconnect: function () {
        ros.close();
        // connection_active = true;
        rosConnectionActive = false;
      },
      isConnected: function () {
        return rosConnectionActive;
        // return false;
      },
      isCommunicated: function () {
        return rosCommunicationActive;
        // return false;
      },
      getState: function () {
        var state = '';
        if(rosConnectionActive) {
          // console.log(communication_active);
          if (rosCommunicationActive) {
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
      isServerConnected: function () { 
        return serverConnected;
        // return true;
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
