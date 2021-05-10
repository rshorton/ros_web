/**
 * @author Russell Toris - rctoris@wpi.edu
 * @author Lars Kunze - l.kunze@cs.bham.ac.uk
 * @author Raffaello Bonghi - raffaello.bonghi@officinerobotiche.it
 */

/**
 * A navigator can be used to add click-to-navigate options to an object. If
 * withOrientation is set to true, the user can also specify the orientation of
 * the robot by clicking at the goal position and pointing into the desired
 * direction (while holding the button pressed).
 *
 * @constructor
 * @param options - object with following keys:
 *   * ros - the ROSLIB.Ros connection handle
 *   * tfClient (optional) - the TF client
 *   * robot_pose (optional) - the robot topic or TF to listen position
 *   * serverName (optional) - the action server name to use for navigation, like '/move_base'
 *   * actionName (optional) - the navigation action name, like 'move_base_msgs/MoveBaseAction'
 *   * rootObject (optional) - the root object to add the click listeners to and render robot markers to
 *   * withOrientation (optional) - if the Navigator should consider the robot orientation (default: false)
 */
NAV2D.Navigator = function(options) {
  var that = this;
  options = options || {};
  var ros = options.ros;
  var tfClient = options.tfClient || null;
  var robot_pose = options.robot_pose || '/robot_pose';
  var serverName = options.serverName || '/move_base';
  var actionName = options.actionName || 'move_base_msgs/MoveBaseAction';
  var withOrientation = options.withOrientation || false;
  var use_image = options.image;
  this.rootObject = options.rootObject || new createjs.Container();

  this.goalMarker = null;
  this.orientationMarker = null;
  this.positionVec3 = null;

  this.objectMarkers = [];
  this.objectMemoryMarkers = [];

  this.searchPathPoseMarkers = [];

  this.planPath = null;
  this.planLocalPath = null;

  that.currentGoalBallPicker = null;

  this.selectedPose = null;
  var currentGoal;

  // setup the actionlib client
  var actionClient = new ROSLIB.ActionClient({
    ros : ros,
    actionName : actionName,
    serverName : serverName
  });

  var showGoalMarker = function(pose) {
    // create a marker for the goal
    if (that.goalMarker === null) {
      if (use_image && ROS2D.hasOwnProperty('NavigationImage')) {
        that.goalMarker = new ROS2D.NavigationImage({
          size: 2.5,
          image: use_image,
          alpha: 0.7,
          pulse: true
        });
      } else {
        that.goalMarker = new ROS2D.NavigationArrow({
          size: 15,
          strokeSize: 1,
          fillColor: createjs.Graphics.getRGB(255, 64, 128, 0.66),
          pulse: true
        });
      }
      that.rootObject.addChild(that.goalMarker);
    }
    that.goalMarker.x = pose.position.x;
    that.goalMarker.y = -pose.position.y;
    that.goalMarker.rotation = stage.rosQuaternionToGlobalTheta(pose.orientation);
    that.goalMarker.scaleX = 1.0 / stage.scaleX;
    that.goalMarker.scaleY = 1.0 / stage.scaleX;
//    that.goalMarker.scaleY = 1.0 / stage.scaleY;
    that.goalMarker.visible = true;
    that.goalMarker.goalId = that.goalId;

    console.log('goalMarker: x,y: ' + that.goalMarker.x + '  ' + that.goalMarker.y +
      ', goal: x,y: ' + pose.position.x + '  ' + pose.position.y);

    var str = 'Goal Position: x,y,yaw: ' + pose.position.x.toFixed(2) + '  ' + pose.position.y.toFixed(2) + '  ' + that.goalMarker.rotation.toFixed(2) + '</br>';
    document.getElementById('goal_position').innerHTML = str;
  };

  var hideGoalMarker = function() {
    that.goalMarker.visible = false;
  };

  /**
   * Send a goal to the navigation stack with the given pose.
   *
   * @param pose - the goal pose
   */
  this.sendGoal = function(pose) {
    // create a goal
    var goal = new ROSLIB.Goal({
      actionClient : actionClient,
      goalMessage : {
        target_pose : {
          header : {
            frame_id : 'map'
          },
          pose : pose
        }
      }
    });
    goal.send();

    that.currentGoal = goal;
    that.pendingGoalCnt = (that.pendingGoalCnt | 0) + 1;

    goal.on('result', function() {
//      that.rootObject.removeChild(that.goalMarker);
      if (that.pendingGoalCnt > 0 && --that.pendingGoalCnt <= 0 ) {
        hideGoalMarker();
      }
    });
  };

  /**
   * Cancel the currently active goal.
   */
  this.cancelGoal = function () {
    if (typeof that.currentGoal !== 'undefined') {
      that.currentGoal.cancel();
    }
  };

  this.startGoal = function () {
    if (that.selectedPose) {
      that.sendGoal(that.selectedPose);
      that.selectedPose = null;
    }
  };

  // get a handle to the stage
  var stage;
  if (that.rootObject instanceof createjs.Stage) {
    stage = that.rootObject;
  } else {
    stage = that.rootObject.getStage();
  }

  // marker for the robot
  var robotMarker = null;
  if (use_image && ROS2D.hasOwnProperty('NavigationImage')) {
    robotMarker = new ROS2D.NavigationImage({
      size: 2.5,
      image: use_image,
      pulse: true
    });
  } else {
    robotMarker = new ROS2D.NavigationArrow({
      size : 25,
      strokeSize : 1,
      fillColor : createjs.Graphics.getRGB(255, 128, 0, 0.66),
      pulse : true
    });
  }

  // wait for a pose to come in first
  robotMarker.visible = false;
  this.rootObject.addChild(robotMarker);
  var initScaleSet = false;

  var updateRobotPosition = function(pose, orientation) {
    // update the robots position on the map
    robotMarker.x = pose.x;
    robotMarker.y = -pose.y;
    if (!initScaleSet) {
      robotMarker.scaleX = 1.0 / stage.scaleX;
      robotMarker.scaleY = 1.0 / stage.scaleX;
//      robotMarker.scaleY = 1.0 / stage.scaleY;
      initScaleSet = true;
    }
    // change the angle
    robotMarker.rotation = stage.rosQuaternionToGlobalTheta(orientation);
    // Set visible
    robotMarker.visible = true;
    console.log('updateRobotPosition: x,y: ' + robotMarker.x + '  ' + robotMarker.y);

    var str = 'Current Position: x,y,yaw: ' + robotMarker.x.toFixed(2) + '  ' + robotMarker.y.toFixed(2) + '  ' + robotMarker.rotation.toFixed(2) + '</br>';
    document.getElementById('cur_position').innerHTML = str;
  };

  ///////////////////////////////////////////////////
  var updateLocalPath = function(plan) {
    if (!this.planLocalPath) {
      this.planLocalPath = new ROS2D.PathShape({path: plan,
                                           strokeSize: 1,
                                           strokeColor: createjs.Graphics.getRGB(255, 0, 0)});
      that.rootObject.addChild(this.planLocalPath);
    } else {
      this.planLocalPath.setPath(plan);
    }
  };

  // setup a listener for the robot pose
  var localPathListener = new ROSLIB.Topic({
    ros: ros,
    name: '/plan',
    messageType: 'nav_msgs/Path',
    throttle_rate: 100,
    use_transient_local: false
  });
  localPathListener.subscribe(function(plan) {
    updateLocalPath(plan);
  });

  ///////////////////////////////////////////////////
  // setup the actionlib client for ball picking
  var actionClientBallPicker = new ROSLIB.ActionClient({
    ros : ros,
    actionName : 'ballpicker_msgs/PickGoalAction',
    serverName : '/ballpicker'
  });

  /**
   * Send a goal to the ballpicker
   */
  this.sendGoalToBallPicker = function(id) {
    // create a goal
    var goal = new ROSLIB.Goal({
      actionClient : actionClientBallPicker,
      goalMessage : {
        id: id
      }
    });
    goal.send();

    that.currentGoalBallPicker = goal;

    goal.on('result', function(res) {
      console.log('ballpicker  result: ' + res);
    });
  };

  this.cancelGoalBallPicker = function () {
    if (typeof that.currentGoalBallPicker !== 'undefined') {
      that.currentGoalBallPicker.cancel();
    }
  };

  this.startGoalBallPicker = function (id) {
     that.sendGoalToBallPicker(id);
  };

  ///////////////////////////////////////////////////
  var showObjectMarker = function(object) {
    // create a marker for the goal
    var obj = that.objectMarkers[object.id];
    if (!obj) {
      obj = new ROS2D.CircleShape({
        size: 3,
      });
      that.objectMarkers[object.id] = obj;
      that.rootObject.addChild(obj);
    }

    obj.x = object.x;
    obj.y = -object.y;
    obj.scaleX = 1.0 / stage.scaleX;
    obj.scaleY = 1.0 / stage.scaleX;
//    obj.scaleY = 1.0 / stage.scaleY;
    obj.visible = true;
    obj.id = object.id;
    obj.life = 5;

    console.log('object: id: ' + obj.id + ', x,y: ' + obj.x + '  ' + obj.y);
  };

  var deleteObjectMarker = function(id) {
    var obj = that.objectMarkers[id];
    if (!obj) {
      that.rootObject.removeChild(obj);
      delete that.objectMarkers[id];
    }
  };

  var updateObjectList = function(list) {
    var str = 'Objects:</br>';
    if (!list || !list.objects) {
      return;
    }
    for (var i = 0; i < list.objects.length; i++) {
      str += 'ID: ' + list.objects[i].id +
             ', (' + list.objects[i].x.toFixed(3) +
             ', ' + list.objects[i].y.toFixed(3) +
             ', ' + list.objects[i].z.toFixed(3) +
             ')</br>';
      showObjectMarker(list.objects[i]);
    }
    document.getElementById('object_info').innerHTML = str;

    // Delete dead objects
    for (var key in that.objectMarkers) {
      if (--that.objectMarkers[key].life < 0) {
        that.rootObject.removeChild(that.objectMarkers[key]);
        delete that.objectMarkers[key];
      }
    }

  };

  // setup a listener for the object list
  var objectListListener = new ROSLIB.Topic({
    ros: ros,
    name: '/object_list_world',
    messageType: 'object_detection_msgs/ObjectDescArray',
    throttle_rate: 100
  });
  objectListListener.subscribe(function(list) {
    updateObjectList(list);
  });

  ///////////////////////////////////////////////////
  var showObjectMemoryMarker = function(object) {
    // create a marker for the goal
    var obj = that.objectMemoryMarkers[object.id];
    if (!obj) {
      obj = new ROS2D.CircleShape({
        size: 4,
        fillColor: createjs.Graphics.getRGB(0, 0, 255)
      });
      that.objectMemoryMarkers[object.id] = obj;
      that.rootObject.addChild(obj);
    }

    obj.x = object.x;
    obj.y = -object.y;
    obj.scaleX = 1.0 / stage.scaleX;
    obj.scaleY = 1.0 / stage.scaleX;
//    obj.scaleY = 1.0 / stage.scaleY;
    obj.visible = true;
    obj.id = object.id;
    obj.life = 1;

    console.log('objectMemory: id: ' + obj.id + ', x,y: ' + obj.x + '  ' + obj.y);
  };

  var deleteObjectMemoryMarker = function(id) {
    var obj = that.objectMemoryMarkers[id];
    if (!obj) {
      that.rootObject.removeChild(obj);
      delete that.objectMemoryMarkers[id];
    }
  };

  var updateObjectMemoryList = function(list) {
    var str = 'ObjectsMemory:</br>';
    if (!list || !list.objects) {
      return;
    }
    for (var i = 0; i < list.objects.length; i++) {
      str += 'ID: ' + list.objects[i].id +
             ', (' + list.objects[i].x.toFixed(3) +
             ', ' + list.objects[i].y.toFixed(3) +
             ', ' + list.objects[i].z.toFixed(3) +
             ')</br>';
      showObjectMemoryMarker(list.objects[i]);
    }
    document.getElementById('object_memory_info').innerHTML = str;

    // Delete dead objects
    for (var key in that.objectMemoryMarkers) {
      if (--that.objectMemoryMarkers[key].life < 0) {
        that.rootObject.removeChild(that.objectMemoryMarkers[key]);
        delete that.objectMemoryMarkers[key];
      }
    }

  };

  // setup a listener for the object list
  var objectMemoryListListener = new ROSLIB.Topic({
    ros: ros,
    name: '/object_memory',
    messageType: 'object_detection_msgs/ObjectDescArray',
    throttle_rate: 100
  });
  objectMemoryListListener.subscribe(function(list) {
    updateObjectMemoryList(list);
  });

  ///////////////////////////////////////////////////
  var updatePath = function(plan) {
    console.log('got path');

    if (!this.planPath) {
      this.planPath = new ROS2D.PathShape({path: plan,
                                           strokeSize: 1,
                                           strokeColor: createjs.Graphics.getRGB(0, 0, 255)});
      that.rootObject.addChild(this.planPath);
    } else {
      this.planPath.setPath(plan);
    }
  };

  // setup a listener for the robot nav path
  var pathListener = new ROSLIB.Topic({
    ros: ros,
    name: '/move_base/NavfnROS/plan',
    messageType: 'nav_msgs/Path',
    throttle_rate: 100,
    use_transient_local: true
  });
  pathListener.subscribe(function(plan) {
    updatePath(plan);
  });

  ///////////////////////////////////////////////////
  // create a marker for the goal
  var showSearchPathMarker = function(pose) {
    var marker = new ROS2D.CircleShape({
         size: 3,
         fillColor: createjs.Graphics.getRGB(0, 0, 255)
    });
    that.searchPathPoseMarkers.push(marker);
    that.rootObject.addChild(marker);

    marker.x = pose.pose.position.x;
    marker.y = -pose.pose.position.y;
    marker.scaleX = 1.0 / stage.scaleX;
    marker.scaleY = 1.0 / stage.scaleX;
    marker.visible = true;
  };

  var deleteAllSearchPathMarkers = function() {
    for (var i = 0; i < that.searchPathPoseMarkers.length; i++) {
      that.rootObject.removeChild(that.searchPathPoseMarkers[i]);
      that.searchPathPoseMarkers[i] = null;
    }
  };

  var showSearchPoints = function(path) {
    console.log('show search path poses');
    for (var i = 0; i < path.poses.length; i++) {
      showSearchPathMarker(path.poses[i]);
    }
  };

  // setup a listener for the robot search path
  var searchPathListener = new ROSLIB.Topic({
    ros: ros,
    name: '/robot_seek_game/search_path',
    messageType: 'nav_msgs/Path',
    throttle_rate: 100,
    use_transient_local: false
  });
  searchPathListener.subscribe(function(path) {
    deleteAllSearchPathMarkers();
    showSearchPoints(path);
  });

  /////////////////////////////////////////////////////

  if(tfClient !== null) {
    tfClient.subscribe(robot_pose, function(tf) {
      updateRobotPosition(tf.translation,tf.rotation);
    });
  } else {
    // setup a listener for the robot pose
    var poseListener = new ROSLIB.Topic({
      ros: ros,
      name: robot_pose,
      messageType: 'geometry_msgs/PoseStamped',
      throttle_rate: 100,
      use_transient_local: false
    });
    poseListener.subscribe(function(pose) {
      updateRobotPosition(pose.pose.position, pose.pose.orientation);
    });
  }

  // Create and advertise the initialpose topic
  that.setInitialPoseTopic = new ROSLIB.Topic({
    ros : ros,
    name : '/initialpose',
    messageType : 'geometry_msgs/PoseWithCovarianceStamped'
//    messageType : 'geometry_msgs/msg/PoseWithCovarianceStamped'
  });
  that.setInitialPoseTopic.advertise();

  var setPose = function(poseIn) {
    var poseMsg = new ROSLIB.Message({
      header : {
        stamp : {
          sec : 0,
          nanosec : 0
        },
        frame_id : 'map'
      },
      pose : {
        pose : poseIn
      }
    });
    that.setInitialPoseTopic.publish(poseMsg);
  };

  that.setInitialPose = function() {
    if (that.selectedPose) {
      setPose(that.selectedPose);
      that.selectedPose = null;
    }
  };


  if (withOrientation === false){
    // setup a double click listener (no orientation)
    this.rootObject.addEventListener('dblclick', function(event) {
      // convert to ROS coordinates
      var coords = stage.globalToRos(event.stageX, event.stageY);
      var pose = new ROSLIB.Pose({
        position : new ROSLIB.Vector3(coords)
      });
      // send the goal
      that.sendGoal(pose);
    });
  } else { // withOrientation === true
    // setup a click-and-point listener (with orientation)
    var position = null;
    var thetaRadians = 0;
    var thetaDegrees = 0;
//    var orientationMarker = null;
    var mouseDown = false;
    var xDelta = 0;
    var yDelta = 0;

    var mouseEventHandler = function(event, mouseState) {

      if (mouseState === 'down'){
        // get position when mouse button is pressed down
        position = stage.globalToRos(event.stageX, event.stageY);
        that.positionVec3 = new ROSLIB.Vector3(position);
        mouseDown = true;

        console.log('mouse down: x,y: ' + that.positionVec3.x + '  ' + that.positionVec3.y);
         if (!that.orientationMarker) {
            if (use_image && ROS2D.hasOwnProperty('NavigationImage')) {
              that.orientationMarker = new ROS2D.NavigationImage({
                size: 2.5,
                image: use_image,
                alpha: 0.7,
                pulse: false
              });
            } else {
              that.orientationMarker = new ROS2D.NavigationArrow({
                size : 25,
                strokeSize : 1,
                fillColor : createjs.Graphics.getRGB(0, 255, 0, 0.66),
                pulse : false
              });
            }
            that.rootObject.addChild(that.orientationMarker);
          }
          that.orientationMarker.visible = true;

          that.orientationMarker.x =  that.positionVec3.x;
          that.orientationMarker.y = -that.positionVec3.y;
          that.orientationMarker.scaleX = 1.0 / stage.scaleX;
          that.orientationMarker.scaleY = 1.0 / stage.scaleX;
//          that.orientationMarker.scaleY = 1.0 / stage.scaleY;

          stage.update();

      }
      else if (mouseState === 'move'){
        // remove obsolete orientation marker

        if ( mouseDown === true) {
          // if mouse button is held down:
          // - get current mouse position
          // - calulate direction between stored <position> and current position
          // - place orientation marker
          var currentPos = stage.globalToRos(event.stageX, event.stageY);
          var currentPosVec3 = new ROSLIB.Vector3(currentPos);

          console.log('mouse move: x,y: ' + that.positionVec3.x + '  ' + that.positionVec3.y);

          xDelta =  currentPosVec3.x - that.positionVec3.x;
          yDelta =  currentPosVec3.y - that.positionVec3.y;

          thetaRadians  = Math.atan2(xDelta,yDelta);

          thetaDegrees = thetaRadians * (180.0 / Math.PI);

          if (thetaDegrees >= 0 && thetaDegrees <= 180) {
            thetaDegrees += 270;
          } else {
            thetaDegrees -= 90;
          }

          that.orientationMarker.x =  that.positionVec3.x;
          that.orientationMarker.y = -that.positionVec3.y;
          that.orientationMarker.rotation = thetaDegrees;
          that.orientationMarker.scaleX = 1.0 / stage.scaleX;
          that.orientationMarker.scaleY = 1.0 / stage.scaleX;
//          that.orientationMarker.scaleY = 1.0 / stage.scaleY;

//          that.rootObject.addChild(that.orientationMarker);
        }
      } else if (mouseDown) { // mouseState === 'up'
        // if mouse button is released
        // - get current mouse position (goalPos)
        // - calulate direction between stored <position> and goal position
        // - set pose with orientation
        // - send goal
        mouseDown = false;

        var goalPos = stage.globalToRos(event.stageX, event.stageY);

        var goalPosVec3 = new ROSLIB.Vector3(goalPos);

        xDelta =  goalPosVec3.x - that.positionVec3.x;
        yDelta =  goalPosVec3.y - that.positionVec3.y;

        thetaRadians  = Math.atan2(xDelta,yDelta);

        if (thetaRadians >= 0 && thetaRadians <= Math.PI) {
          thetaRadians += (3 * Math.PI / 2);
        } else {
          thetaRadians -= (Math.PI/2);
        }

        var qz =  Math.sin(-thetaRadians/2.0);
        var qw =  Math.cos(-thetaRadians/2.0);

        var orientation = new ROSLIB.Quaternion({x:0, y:0, z:qz, w:qw});

        var pose = new ROSLIB.Pose({
          position :    that.positionVec3,
          orientation : orientation
        });

        showGoalMarker(pose);

        that.selectedPose = pose;

        // send the goal
//        that.sendGoal(pose);

        that.orientationMarker.visible = false;
//        that.rootObject.removeChild(that.orientationMarker);
        stage.update();
      }
    };

    this.rootObject.addEventListener('stagemousedown', function(event) {
      mouseEventHandler(event,'down');
    });

    this.rootObject.addEventListener('stagemousemove', function(event) {
      mouseEventHandler(event,'move');
    });

    this.rootObject.addEventListener('stagemouseup', function(event) {
      mouseEventHandler(event,'up');
    });
  }
  return this;
};

