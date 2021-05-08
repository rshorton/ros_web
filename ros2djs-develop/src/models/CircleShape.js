/**
 * @author Bart van Vliet - bart@dobots.nl
 */

/**
 * A circle.
 *
 * @constructor
 * @param options - object with following keys:
 *   * size (optional) - the size of the marker
 *   * strokeSize (optional) - the size of the outline
 *   * strokeColor (optional) - the createjs color for the stroke
 *   * fillColor (optional) - the createjs color for the fill
 */
ROS2D.CircleShape = function(options) {
	var that = this;
	options = options || {};
	var size = options.size || 40;
	var strokeSize = options.strokeSize || 1;
	var strokeColor = options.strokeColor || createjs.Graphics.getRGB(0, 0, 0);
	var fillColor = options.fillColor || createjs.Graphics.getRGB(255, 255, 0);

	// draw the circle
	var g = new createjs.Graphics();

  g.setStrokeStyle(strokeSize, 'round', 'round');
  g.beginStroke('#000');
  g.beginFill(fillColor);
  g.drawCircle(0, 0, size);

	// create the shape
	createjs.Shape.call(this, g);
};
ROS2D.CircleShape.prototype.__proto__ = createjs.Shape.prototype;
