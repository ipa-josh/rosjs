
if(!rosjs.Time) rosjs.Time = function(secs) {
	this.now = function() {
		return new rosjs.Time(getTime()/1000.);
	};

	return {
		time:secs,
		valueOf: function() {return this.time;},
		to_sec: function() {return this.time;},
		to_nsec: function() {return this.time*1000*1000;}
	};
};