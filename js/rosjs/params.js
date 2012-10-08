
if(!rosjs.get_param) rosjs.get_param = function(name) {
	return rosjs.ServiceProxy("/get_param")(name);
};

if(!rosjs.set_param) rosjs.set_param = function(name, value) {
	rosjs.ServiceCall("/set_param", rosjs.nop, value);
};


if(!rosjs.has_param) rosjs.has_param = function(name) {
};

if(!rosjs.delete_param) rosjs.delete_param = function(name) {
};

if(!rosjs.get_param_names) rosjs.get_param_names = function() {
};
