#include "flexfly_optical_switch.h"
#include <sstmac/hardware/common/connection.h>
#include <iostream>


namespace sstmac {
namespace hw {
	flexfly_optical_switch::flexfly_optical_switch(sprockit::sim_parameters* params,
    uint64_t id,
    event_manager* mgr) : optical_switch(params, id, mgr) {
		//init_connection(params); // this has to be called upon class initialization
		//std
	};

	flexfly_optical_switch::~flexfly_optical_switch() {

	}

	void flexfly_optical_switch::init(unsigned int phase) {
		init(phase);
		//connection::init(phase);
		return; 
	};

	void flexfly_optical_switch::setup() {
		//init(phase);
		//connection::init(phase);makes
		return; 
	};

	void flexfly_optical_switch::connect_input(sprockit::sim_parameters* params, 
                              					int src_outport, 
                              					int dst_inport,
                              					event_handler* credit_handler) {
		inport_handler_[src_outport] = credit_handler;
		return;
	};

	void flexfly_optical_switch::connect_output(sprockit::sim_parameters* params, 
                              					int src_outport, 
                              					int dst_inport,
                              					event_handler* payload_handler) {
		outport_handler_[src_outport] = payload_handler;
		return;
	};

	link_handler* flexfly_optical_switch::payload_handler(int port) const {
		return new_link_handler(this, &flexfly_optical_switch::recv_payload);
	};

	link_handler* flexfly_optical_switch::credit_handler(int port) const {
		return new_link_handler(this, &flexfly_optical_switch::recv_credit);
	};

	void flexfly_optical_switch::deadlock_check() {
		return;
	};

	void flexfly_optical_switch::deadlock_check(event* ev) {
		return;
	};

	void flexfly_optical_switch::recv_credit(event* ev) {
		return;
	};

	void flexfly_optical_switch::recv_payload(event* ev) {
		return;
	};
}
}