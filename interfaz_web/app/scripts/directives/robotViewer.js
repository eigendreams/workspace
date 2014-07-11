'use strict';


console.log("Si entro");

angular.module('finderApp')

  /**
   * Removes server error when user updates input
   */
  .directive('robotViewer', function () {
    return {
      restrict: 'EA',
      scope: {
        data: '=data'
      },
      templateUrl: '/views/templates/robot_viewer.html',
      compile: function (tElem, attrs) {
        // tElem.css('height', '100px');
        // tElem.css('width', '100px');
        // tElem.text('Hola que onda');

        var scene = new THREE.Scene();
        var camera = new THREE.PerspectiveCamera( 75, tElem.innerWidth / tElem.innerHeight, 0.1, 10000 );

        var renderer = new THREE.CanvasRenderer();
        renderer.setSize( '200px', '200px' );
        tElem.append( renderer.domElement );

        var geometry = new THREE.CubeGeometry(1,1,1);
        var material = new THREE.MeshBasicMaterial( { color: 0x00ff00 } );
        var cube = new THREE.Mesh( geometry, material );

        scene.add( cube );
        // camera.position.z = 10;

        return function (scope, elem, attrs) {
          function render() {
            requestAnimationFrame(render);
            renderer.render(scene, camera);
            // console.log("Rendering");
          }

          render();
   
        };
      }
      // template
      // link: function(scope, element, attrs, ngModel) {
        // element.on('keydown', function() {
          // return ngModel.$setValidity('mongoose', true);
        // });
      // },
      // template: 'Hola mundo'
    };
  });