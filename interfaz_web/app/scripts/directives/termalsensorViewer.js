'use strict';

angular.module('finderApp')

  /**
   * Removes server error when user updates input
   */
  .directive('termalsensorViewer', function () {
    return {
      restrict: 'E',
      scope: {
        data: '=data'
      },
      templateUrl: '/views/templates/termalsensor_viewer.html'
      
      // template
      // link: function(scope, element, attrs, ngModel) {
        // element.on('keydown', function() {
          // return ngModel.$setValidity('mongoose', true);
        // });
      // },
      // template: 'Hola mundo'
    };
  });