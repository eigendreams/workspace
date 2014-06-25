'use strict';

angular.module('finderApp')
  .factory('Session', function ($resource) {
    return $resource('/api/session/');
  });
