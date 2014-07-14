'use strict';

angular.module('finderApp')
  .factory('Ros', ['$rootScope', '$interval', '$timeout', '$http', function ($rootScope, $interval, $timeout, $http) {

    // var serverIP = "192.168.88.247";
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
    var mapViewer;
    var gridClient;


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
            // topics.leftOut.topic.unsubscribe();
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
            // topics.rightOut.topic.unsubscribe();
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
            // topics.brOut.topic.unsubscribe();
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
            // topics.blOut.topic.unsubscribe();
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
            // topics.frOut.topic.unsubscribe();
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
            // topics.flOut.topic.unsubscribe();
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
            // topics.baseOut.topic.unsubscribe();
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
            // topics.armOut.topic.unsubscribe();
          });
        }
      },
      forearmOut: {
        name: '/forearm_out',
        type: 'std_msgs/Int16',
        value: 0,
        active: false,
        subscribe: function () {
          this.topic.subscribe( function (message) {
            topics.forearmOut.value = message.data;
            topics.forearmOut.active = false;
            // topics.armOut.topic.unsubscribe();
          });
        }
      },
      wristOut: {
        name: '/wrist_out',
        type: 'std_msgs/Int16',
        value: 0,
        active: false,
        subscribe: function () {
          this.topic.subscribe( function (message) {
            topics.wristOut.value = message.data;
            topics.wristOut.active = false;
            // topics.armOut.topic.unsubscribe();
          });
        }
      },
      wristDes: {
        name: '/wrist_des',
        type: 'std_msgs/Int16',
        value: 0,
        active: false,
        subscribe: function () {
          this.topic.subscribe( function (message) {
            topics.wristDes.value = message.data;
            topics.wristDes.active = false;
            // topics.armOut.topic.unsubscribe();
          });
        }
      },
      palmOut: {
        name: '/palm_out',
        type: 'std_msgs/Int16',
        value: 0,
        active: false,
        subscribe: function () {
          this.topic.subscribe( function (message) {
            topics.palmOut.value = message.data;
            topics.palmOut.active = false;
            // topics.armOut.topic.unsubscribe();
          });
        }
      },
      palmDes: {
        name: '/palm_des',
        type: 'std_msgs/Float32',
        value: 0,
        active: false,
        subscribe: function () {
          this.topic.subscribe( function (message) {
            topics.palmDes.value = message.data;
            topics.palmDes.active = false;
            // topics.armOut.topic.unsubscribe();
          });
        }
      },
      gripperOut: {
        name: '/gripper_out',
        type: 'std_msgs/Int16',
        value: 0,
        active: false,
        subscribe: function () {
          this.topic.subscribe( function (message) {
            topics.gripperOut.value = message.data;
            topics.gripperOut.active = false;
            // topics.armOut.topic.unsubscribe();
          });
        }
      },
      gripperDes: {
        name: '/gripper_des',
        type: 'std_msgs/Float32',
        value: 0,
        active: false,
        subscribe: function () {
          this.topic.subscribe( function (message) {
            topics.gripperDes.value = message.data;
            topics.gripperDes.active = false;
            // topics.armOut.topic.unsubscribe();
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
            // topics.baseDes.topic.unsubscribe();
          });
        }
      },
      irOut: {
        name: '/ir_out',
        type: 'std_msgs/String',
        value: [[0,0,0,0],
                [0,0,0,0],
                [0,0,0,0],
                [0,0,0,0],
                [0,0,0,0],
                [0,0,0,0],
                [0,0,0,0],
                [0,0,0,0],
                [0,0,0,0],
                [0,0,0,0],
                [0,0,0,0],
                [0,0,0,0],
                [0,0,0,0],
                [0,0,0,0],
                [0,0,0,0],
                [0,0,0,0]],
        active: false,
        subscribe: function () {
          this.topic.subscribe( function (message) {
            topics.irOut.value = message.data;
            topics.irOut.active = false;
            //for (var element in message.data) {
              //console.log(element);

            //}
            var index = -1;
            topics.irOut.value = [[],[],[],[],[],[],[],[],[],[],[],[],[],[],[],[]];
            for (var i=0; i<message.data.length; i++) {
              if (i%4 === 0) {
                index++;
              } 
              topics.irOut.value[15-index][i%4] = message.data[i].charCodeAt(0);
              //console.log(index);
              //console.log(mes);
            }
            // console.log(topics.irOut.value);
            // topics.baseDes.topic.unsubscribe();
          });
        }
      },
      co2: {
        name: '/co2',
        type: 'std_msgs/Int8',
        value: 0,
        active: false,
        subscribe: function () {
          this.topic.subscribe( function (message) {
            topics.co2.value = message.data;
            topics.co2.active = false;
            // topics.baseDes.topic.unsubscribe();
          });
        }
      },
      pitch: {
        name: '/pitch',
        type: 'std_msgs/Int8',
        value: 0,
        active: false,
        subscribe: function () {
          this.topic.subscribe( function (message) {
            topics.pitch.value = message.data;
            topics.pitch.active = false;
            // topics.baseDes.topic.unsubscribe();
          });
        }
      },
      roll: {
        name: '/roll',
        type: 'std_msgs/Int8',
        value: 0,
        active: false,
        subscribe: function () {
          this.topic.subscribe( function (message) {
            topics.roll.value = message.data;
            topics.roll.active = false;
            // topics.baseDes.topic.unsubscribe();
          });
        }
      },
      batteryLevel: {
        name: '/battery_level',
        type: 'std_msgs/Int16',
        value: 0,
        active: false,
        subscribe: function () {
          this.topic.subscribe( function (message) {
            topics.batteryLevel.value = message.data;
            topics.batteryLevel.active = false;
            // topics.baseDes.topic.unsubscribe();
          });
        }
      },
      volt: {
        name: '/volt',
        type: 'std_msgs/Int16',
        value: 0,
        active: false,
        subscribe: function () {
          this.topic.subscribe( function (message) {
            topics.batteryLevel.value = message.data;
            topics.batteryLevel.active = false;
            // topics.baseDes.topic.unsubscribe();
          });
        }
      },
      addVictim: {
        name: '/add_victim',
        type: 'std_msgs/Int16',
        value: 0,
        active: false,
        subscribe: function () {},
        publish: function (data) {
          var message = new ROSLIB.Message ({
            data: 1
          });
          this.topic.publish(message);
          console.log("Se va a publicar en victim");
          console.log(message);
          $timeout( function () {
            var message = new ROSLIB.Message ({
              data: 0
            });
            topics.addVictim.topic.publish(message);
          }, 5000);
        }
      },
      leds: {
        name: '/leds',
        type: 'std_msgs/Int16',
        value: 0,
        active: false,
        subscribe: function () {},
        publish: function (data) {
          var message = new ROSLIB.Message ({
            data: data
          });
          console.log("Se va a publicar en leds " + message);
          this.topic.publish(message);
        }
      }

    };

    var getNodes = function () {
      $http.get('/api/getNodes').
        success(function(data) {
          nodes = data;
          // console.log(data);

          if (serverConnected === false) {
            serverConnected = true;
            $rootScope.$broadcast('serverStateChanged');
          }
          $timeout.cancel(serverConnectionPromise);
          serverConnectionPromise = $timeout( function () {
            for (var node in nodes) {
              nodes[node] = '0';
            }
            serverConnected = false;
            $rootScope.$broadcast('serverStateChanged');
          }, 3000);
        }).
        error(function(data) {
          console.log("Error de conexión"); 
        });
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

        topics[key].subscribe();
      }

      // topics.irOut.subscribe();

      // topics.alive.subscribe();

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
        publishData: function (topic, data) {
          console.log("Se va a publicar data");
          topics[topic].publish(data);
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
