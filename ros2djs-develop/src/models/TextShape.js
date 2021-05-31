/**
 * A Text object.
 *
 * @constructor
 * @param options - object with following keys:
 *   * fillColor (optional) - the createjs color for the fill
 */
ROS2D.TextShape = function(options) {
	var that = this;
	options = options || {};
	var color = options.fillColor || createjs.Graphics.getRGB(255, 255, 0);
  var text = options.text;
  var font = options.font || '20px Arial';

  //console.log('Text shape, text= ' + text);
  createjs.Text.call(this, text, font, color);
};
ROS2D.TextShape.prototype.__proto__ = createjs.Text.prototype;
