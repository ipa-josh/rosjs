
;if(!rosjs) rosjs = {
		connection: null,
		id: 0,
		registered_objs: {},
		registered_srvs: {},
		console: {
			info: function() {},
			warning: function() {},
			error: function() {}
		},

		//constants
		NOT_INITIALIZED: -1,
		NOT_CONNECTED: -2,
		CONNECTED: 2,
		OK: 1,

		PUBLISHER: 1,
		SUBSCRIBER: 2,
		SERVICE_RESP: 3,

		INFO:'info',
		WARNING: 'warning',
		ERROR: 'error', 
		NONE: 'none',

		status: NOT_CONNECTED,

		init: function(ROS_MASTER_URI) {
			if(!ROS_MASTER_URI)
				ROS_MASTER_URI = location.hostname.substring(0,location.hostname.indexOf("."))+":9090";
			alert(ROS_MASTER_URI);
			this.connection = new WebSocket("ws://"+ROS_MASTER_URI);
			this.connection.onmessage = rosjs.onMessage;

			this.socket.onerror = rosjs.onError;
			this.socket.onopen = rosjs.onOpen;
			this.socket.onclose = rosjs.onClose;

		},
		onError: function(event) {
			rosjs.ROS_ERROR("rosjs error on websocket\n"+event);
		},
		onOpen: function(event) {
			rosjs.status = CONNECTED;
			rosjs.ROS_INFO("connect to ROS master\n"+event);
		},
		onClose: function(event) {
			rosjs.status = NOT_CONNECTED;
			rosjs.ROS_INFO("disconnect from ROS master\n"+event);
		},
		receiveMessage: function(event) {
			rosjs.__log("received message");

			msg = JSON.parse(event.data);

			try {
				switch(msg.op) {
				case "set_level": rosjs.__set_level(msg.level); break;
				case "status":
					switch(msg.level) {
					case rosjs.INFO: rosjs.console.info(msg.msg); break;
					case rosjs.WARNING: rosjs.console.warning(msg.msg); break;
					case rosjs.ERROR: rosjs.console.error(msg.msg); break;
					default:
						throw "unknown level for status ("+msg.level+")";
					}
					break;
				case "publish": rosjs.__onPublish(msg); break;
				case "service_response": rosjs.__onServiceResponse(msg); break;
				default:
					throw "unknown operation ("+msg.op+")";
				}
			}
			catch(e) {
				rosjs.ROS_ERROR("rosjs exception on receiveMessage: "+e);
			}
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
		__registerIDSrv: function(id, cb) {
			registered_srvs[id] = cb;
		},
		__onPublish: function(msg) {
			if(registered_objs[msg.id].type!=rosjs.SUBSCRIBER)
				throw "object did not subscribe";

			try {
				registered_objs[msg.id].callback(msg);
			}
			catch(e) {
				rosjs.ROS_ERROR("rosjs exception on subscriber: "+e);
			}

		},
		__onServiceResponse: function(msg) {
			if(!registered_srvs[msg.id])
				throw "object did not provide a service responce callback";

			try {
				registered_srvs[msg.id](msg);
			}
			catch(e) {
				rosjs.ROS_ERROR("rosjs exception on service: "+e);
			}

			delete registered_srvs[msg.id];
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
		},

		ServiceCall: function(service, callback, data, options)
		{
			var id = rosjs.__getNextID();
			var obj = {
					'op': 'call_service',
					'service': this.topic_name,
					'id': id
			};
			if(data) obj['msg'] = data;

			var add = ['fragment_size','compression'];
			for (a in add)
				if(this.options && this.options[a])
					obj[a] =  this.options[a];

			rosjs.__sendJSON(obj);

			rosjs.__registerIDSrv(r, callback);
		},

		ServiceProxy: function(service, options)
		{
			var id = rosjs.__getNextID();
			var obj = {
					'op': 'call_service',
					'service': this.topic_name,
					'id': id
			};

			var add = ['fragment_size','compression'];
			for (a in add)
				if(this.options && this.options[a])
					obj[a] =  this.options[a];

			var r = function() {
				this.obj = obj;
				this.done = false;
				this.callback = function(data) {
					this.data = data;
					this.done = true;
				};

				rosjs.__sendJSON(obj);

				/*waitforValue() {
					var rv = do_request_to_database_server();
				}
				or {
					hold(3000);
					throw ("timeout in service call");
				}*/
				while(!this.done);
				
				return this.data;
			};

			rosjs.__registerIDSrv(r, r.callback);

			return r;
		},

		nop: function() {}
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