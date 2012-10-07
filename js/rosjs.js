
;if(!rosjs) rosjs = {
		connection: null,
		id: 0,
		registered_objs: {},

		//constants
		NOT_INITIALIZED: -1,
		OK: 1,

		PUBLISHER: 1,
		SUBSCRIBER: 2,
		SERVICE: 3,
		
		INFO:'info',
		WARNING: 'warning',
		ERROR: 'error', 
		NONE: 'none',

		init: function(ROS_MASTER_URI) {
			if(!ROS_MASTER_URI)
				ROS_MASTER_URI = location.hostname.substring(0,location.hostname.indexOf("."))+":9090";
			alert(ROS_MASTER_URI);
			this.connection = new WebSocket("ws://"+ROS_MASTER_URI);
			this.connection.onmessage = rosjs.onMessage;
		},
		onMessage: function(incoming_message) {
			alert(incoming_message);
			console.log("Received:", incoming_message.data);
		},

		__sendJSON: function(obj) {
			this.connection.send(JSON.stringify(obj));
		},
		__getNextID: function() {
			return this.id++;
		},
		__registerID: function(obj) {
			registered_objs[obj.id] = obj;
		},
		
		setStatusLevel: function(lvl) {
			rosjs.__sendJSON({ op: 'set_level', level: lvl });
		},
		
		ROS_ERROR: function(msg) {
			rosjs.__sendJSON({ op: 'status', level: rosjs.ERROR, msg: msg });
		},
		ROS_WARNING: function(msg) {
			rosjs.__sendJSON({ op: 'status', level: rosjs.WARNING, msg: msg });
		},
		ROS_INFO: function(msg) {
			rosjs.__sendJSON({ op: 'status', level: rosjs.INFO, msg: msg });
		},

		Publisher: function(topic_name, message_type)
		{
			var r = {
					topic_name: topic_name,
					message_type: message_type,
					id: rosjs.__getNextID(),
					status: rosjs.NOT_INITIALIZED,
					type: rosjs.PUBLISHER,

					publish: function(data) {
						rosjs.__sendJSON({'op': 'publish', 'topic': this.topic_name, 'msg': data, 'id': this.id}); 
					},

					advertise: function(data) {
						rosjs.__sendJSON({'op': 'advertise', 'topic': this.topic_name, 'type': this.message_type, 'id': this.id}); 
					},

					unadvertise: function(data) {
						rosjs.__sendJSON({'op': 'unadvertise', 'topic': this.topic_name, 'id': this.id}); 
					}

			};
			rosjs.__registerID(r);

			r.advertise(data);

			return r;
		},

		Subscriber: function(topic_name, message_type, callback, options)
		{
			var r = {
					topic_name: topic_name,
					message_type: message_type,
					callback: callback,
					options: options,
					
					id: rosjs.__getNextID(),
					status: rosjs.NOT_INITIALIZED,
					type: rosjs.SUBSCRIBER,

					subscribe: function() {
						var obj = {
								'op': 'subscribe',
								'topic': this.topic_name,
								'type': this.message_type,
								'id': this.id
						};
						
						var add = ['throttle_rate','queue_length','fragment_size','compression'];
						for (a in add)
							if(this.options && this.options[a])
								obj[a] =  this.options[a];
						
						rosjs.__sendJSON(obj);
					},

					unsubscribe: function() {
						rosjs.__sendJSON({'op': 'unsubscribe', 'topic': this.topic_name, 'id': this.id});
					}
			};
			rosjs.__registerID(r);

			r.subscribe();

			return r;
		}
};

rosjs.init();

if(!ROS_ERROR) ROS_ERROR = function(msg) {
	rosjs.ROS_ERROR(msg);
};
if(!ROS_WARNING) ROS_WARNING = function(msg) {
	rosjs.ROS_WARNING(msg);
};
if(!ROS_INFO) ROS_INFO = function(msg) {
	rosjs.ROS_INFO(msg);
};