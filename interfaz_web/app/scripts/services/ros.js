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
    var rosCommunicationPromise;
    var serverConnectionPromise;

    var topics = {
      alive: {
        name: '/alive',
        type: 'std_msgs/Int16',
        value: 0,
        active: false,
        subscribe: function () {
          this.topic.subscribe( function (message) {
            if (rosCommunicationActive === false) {
              rosCommunicationActive = true;
              $rootScope.$broadcast('rosStateChanged');
            }
            // rosCommunicationActive = true;
            $timeout.cancel(rosCommunicationPromise);
            rosCommunicationPromise = $timeout( function () {
              rosCommunicationActive = false;
              $rootScope.$broadcast('rosStateChanged');
            }, 3000);
          });
        }
      },
      leftOut: {
        name: '/left_out',
        type: 'std_msgs/Int16',
        value: 0,
        active: false,
        subscribe: function () {
          this.topic.subscribe( function (message) {
            topics.leftOut.value = message.data;
            topics.leftOut.active = false;
            topics.leftOut.topic.unsubscribe();
          });
        }
      },
      rightOut: {
        name: '/right_out',
        type: 'std_msgs/Int16',
        value: 0,
        active: false,
        subscribe: function () {
          this.topic.subscribe( function (message) {
            topics.rightOut.value = message.data;
            topics.rightOut.active = false;
            topics.rightOut.topic.unsubscribe();
          });
        }
      },
      brOut: {
        name: '/br_out',
        type: 'std_msgs/Int16',
        value: 0,
        active: false,
        subscribe: function () {
          this.topic.subscribe( function (message) {
            topics.brOut.value = message.data;
            topics.brOut.active = false;
            topics.brOut.topic.unsubscribe();
          });
        }
      },
      blOut: {
        name: '/bl_out',
        type: 'std_msgs/Int16',
        value: 0,
        active: false,
        subscribe: function () {
          this.topic.subscribe( function (message) {
            topics.blOut.value = message.data;
            topics.blOut.active = false;
            topics.blOut.topic.unsubscribe();
          });
        }
      },
      frOut: {
        name: '/fr_out',
        type: 'std_msgs/Int16',
        value: 0,
        active: false,
        subscribe: function () {
          this.topic.subscribe( function (message) {
            topics.frOut.value = message.data;
            topics.frOut.active = false;
            topics.frOut.topic.unsubscribe();
          });
        }
      },
      flOut: {
        name: '/fl_out',
        type: 'std_msgs/Int16',
        value: 0,
        active: false,
        subscribe: function () {
          this.topic.subscribe( function (message) {
            topics.flOut.value = message.data;
            topics.flOut.active = false;
            topics.flOut.topic.unsubscribe();
          });
        }
      },
      baseOut: {
        name: '/base_out',
        type: 'std_msgs/Int16',
        value: 0,
        active: false,
        subscribe: function () {
          this.topic.subscribe( function (message) {
            topics.baseOut.value = message.data;
            topics.baseOut.active = false;
            topics.baseOut.topic.unsubscribe();
          });
        }
      },
      armOut: {
        name: '/arm_out',
        type: 'std_msgs/Int16',
        value: 0,
        active: false,
        subscribe: function () {
          this.topic.subscribe( function (message) {
            topics.armOut.value = message.data;
            topics.armOut.active = false;
            topics.armOut.topic.unsubscribe();
          });
        }
      },
      baseDes: {
        name: '/base_des',
        type: 'std_msgs/Float32',
        value: 0,
        active: false,
        subscribe: function () {
          this.topic.subscribe( function (message) {
            topics.baseDes.value = message.data;
            topics.baseDes.active = false;
            topics.baseDes.topic.unsubscribe();
          });
        }
      }
    };

    var getNodes = function () {
      $http.get('/api/getNodes').
        success(function(data) {
          nodes = data;

          if (serverConnected === false) {
            serverConnected = true;
            $rootScope.$broadcast('serverStateChanged');
          }
          $timeout.cancel(serverConnectionPromise);
          serverConnectionPromise = $timeout( function () {
            serverConnected = false;
            $rootScope.$broadcast('serverStateChanged');
          }, 3000);
        })
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
    }, 1200);

    var init = function () {

      for (var topic in topics) {
        var key = topic;
        topics[key].topic = new ROSLIB.Topic({
          ros : ros,
          name : topics[key].name,
          messageType : topics[key].type
        });
      }

      topics.alive.subscribe();

    };

    return {
      ros: ros,
      getServerIP: function () { 
        return serverIP; 
      },
      connect: function () {
        ros.close();
        ros.connect('ws://' + serverIP + ':9090');
        if (rosConnectionActive === false) {
          rosConnectionActive = true;
          $rootScope.$broadcast('rosStateChanged');
        }
        // rosConnectionActive = true;
        init();
        // connection_active = true;
      },
      disconnect: function () {
        ros.close();
        // connection_active = true;
        if (rosConnectionActive === true) {
          rosConnectionActive = false;
          $rootScope.$broadcast('rosStateChanged');
        }
        // rosConnectionActive = false;
      },
      getRosState: function () {
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
      getServerState: function () { 
        if (serverConnected) {
          return 'Connected';
        } else {
          return 'Disconnected';
        }
      },
      node: {
        start : function (node) {
          $http.post('/api/startNode', {node: node}).
            success(function(data) {
              getNodes();
          });
        },
        stop : function (node) {
          $http.post('/api/stopNode', {node: node}).
            success(function(data) {
              getNodes();
            });
        },
        getNodes : function () {
          return nodes;
        }
      },
      topic: {
        updateData: function (topic) {
          if (rosCommunicationActive) {
            // if (topics[topic].active === false) {
              topics[topic].topic.unsubscribe();
              topics[topic].topic = new ROSLIB.Topic({
                ros : ros,
                name : topics[topic].name,
                messageType : topics[topic].type
              });
              topics[topic].active = true;
              topics[topic].subscribe();
            // } else {
            //   console.log("Listener todavía activo");
            // }
          } else {
            console.log("No hay comunicación con Ros");
          }
        },
        getData: function (topic) {
          return topics[topic].value;
        },
        getNames: function () {
          var topicNames = {};
          for (var key in topics) {
            topicNames[key] = 0;
          }
          return topicNames;
        },
        unsubscribe: function (topic) {
          // topics[topic].unsubscribe();
          if (rosCommunicationActive) {
            topics[topic].topic.unsubscribe();
          } else {
            console.log("No hay comunicación con ros");
          }
          // topics[topic].topic = new ROSLIB.Topic({
          //   ros : ros,
          //   name : topics[topic].name,
          //   messageType : topics[topic].type
          // });
        }
      }
    };
  }]);
