'use strict';

angular.module('finderApp', [
  'ngCookies',
  'ngResource',
  'ngSanitize',
  'ngRoute'
])
  .config(function ($routeProvider, $locationProvider, $httpProvider) {
    $routeProvider
      .when('/', {
        templateUrl: 'partials/main',
        controller: 'MainCtrl'
      })
      .when('/login', {
        templateUrl: 'partials/login',
        controller: 'LoginCtrl'
      })
      .when('/signup', {
        templateUrl: 'partials/signup',
        controller: 'SignupCtrl'
      })
      .when('/settings', {
        templateUrl: 'partials/settings',
        controller: 'SettingsCtrl',
        authenticate: true
      })
      .when('/ros_test', {
        templateUrl: 'partials/ros_test',
        controller: 'RosTestCtrl',
        authenticate: false
      })
      .when('/ros_nodes', {
        templateUrl: 'partials/ros_nodes',
        // controller: 'RosNodesCtrl',
        authenticate: false
      })
      .when('/home', {
        templateUrl: 'partials/home',
        controller: 'HomeCtrl',
        authenticate: false
      })
      .when('/topics', {
        templateUrl: 'partials/topics',
        controller: 'TopicsCtrl',
        authenticate: false
      })
      .otherwise({
        redirectTo: '/'
      });
      
    $locationProvider.html5Mode(true);
      
    // Intercept 401s and redirect you to login
    $httpProvider.interceptors.push(['$q', '$location', function($q, $location) {
      return {
        'responseError': function(response) {
          if(response.status === 401) {
            $location.path('/login');
            return $q.reject(response);
          }
          else {
            return $q.reject(response);
          }
        }
      };
    }]);
  })
  .run(function ($rootScope, $location, Auth) {

    // Redirect to login if route requires auth and you're not logged in
    $rootScope.$on('$routeChangeStart', function (event, next) {
      
      if (next.authenticate && !Auth.isLoggedIn()) {
        $location.path('/login');
      }
    });

    // Creation of the ros system and things


    // $rootScope.ros = new ROSLIB.Ros();
    // $rootScope.ros.connect('ws://localhost:9090');
    // $rootScope.datas = "hola";
  });

