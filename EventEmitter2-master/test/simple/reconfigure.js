var simpleEvents = require('nodeunit').testCase;
var file = '../../lib/eventemitter2';
var EventEmitter2;

if(typeof require !== 'undefined') {
  EventEmitter2 = require(file).EventEmitter2;
}
else {
  EventEmitter2 = window.EventEmitter2;
}

module.exports = simpleEvents({

  'reconfigure1. initialize, removeAllListeners' : function (test) {

    var emitter,
        config = {
          wildcard: true, // should the event emitter use wildcards.
          delimiter: '::::', // the delimiter used to segment namespaces, defaults to `.`.
          maxListeners: 20 // the max number of listeners that can be assigned to an event, defaults to 10.
      };

    emitter = new EventEmitter2(config);

    emitter.removeAllListeners();

    test.equal(emitter._maxListeners, config.maxListeners, 'should be ' + config.maxListeners);

    test.equal(emitter._conf.maxListeners, config.maxListeners, 'should be ' + config.maxListeners);
    test.equal(emitter._conf.delimiter, config.delimiter, 'should be ' + config.delimiter);
    test.equal(emitter._conf.wildcard, config.wildcard, 'should be ' + config.wildcard);

    test.expect(4);
    test.done();
  },

  'reconfigure1. setMaxListeners, removeAllListeners' : function (test) {
    var emitter,
        amount = 99;

    emitter = new EventEmitter2();

    emitter.setMaxListeners(amount);

    emitter.removeAllListeners();

    test.equal(emitter._maxListeners, amount, 'should be ' + amount);

    test.equal(emitter._conf.maxListeners, amount, 'should be ' + amount);

    test.expect(2);
    test.done();
  },

  'getMaxListeners': function (test) {
    var emitter = new EventEmitter2(),
        amount = 10; //default amount

    test.equal(emitter.getMaxListeners(), amount, 'should be ' + amount);

    amount= 99;

    emitter.setMaxListeners(amount);

    test.equal(emitter.getMaxListeners(), amount, 'should be ' + amount);

    test.done();
  },

  'defaultMaxListeners': function (test) {
    var defaultAmount = 10,
        amount = defaultAmount;

    test.equal(EventEmitter2.defaultMaxListeners, amount, 'should be ' + amount);
    amount = 99;
    EventEmitter2.defaultMaxListeners = amount;
    test.equal(EventEmitter2.defaultMaxListeners, amount, 'should be ' + amount);
    EventEmitter2.defaultMaxListeners = defaultAmount; // rollback

    test.done();
  }

});
