#include "flexfly_optical_switch.h"
#include <sstmac/hardware/common/connection.h>
#include <iostream>


namespace sstmac {
namespace hw {
	flexfly_optical_switch::flexfly_optical_switch(sprockit::sim_parameters* params,
    												uint64_t id,
    												event_manager* mgr) : optical_switch(params, id, mgr) {
		init_links(params); // this has to be called upon class initialization
		num_ports_ = params->get_int_param("optical_switch_radix");
		//outport_handler_ = new std::vector<event_handler*>(num_ports_);
		//inport_handler_ = new std::vector<event_handler*>(num_ports_);
		inout_connection_ = new int[num_ports_];
		for (int i = 0; i < num_ports_; i++) {
			inout_connection_[i] = i;
		}
	};

	flexfly_optical_switch::~flexfly_optical_switch() {
		//std::cout << "FLEXFLY_OPTICAL_SWITCH DECONSTRUCTOR" << std::endl;
		delete [] inout_connection_;
	}
	/*
	void flexfly_optical_switch::init(unsigned int phase) {
		init(phase);
		//connection::init(phase);
		return; 
	};
	*/
	void flexfly_optical_switch::setup() {
		//init(phase);
		//connection::init(phase);makes
		return; 
	};

	void flexfly_optical_switch::connect_input(sprockit::sim_parameters* params, 
                              					int src_outport, 
                              					int dst_inport,
                              					event_handler* credit_handler) {
		std::cout << "input src_outport: " << std::to_string(src_outport) << " and dst_inport: " << std::to_string(dst_inport) << std::endl;
 		inport_handler_[2] = credit_handler;
		return;
	};

	void flexfly_optical_switch::connect_output(sprockit::sim_parameters* params, 
                              					int src_outport, 
                              					int dst_inport,
                              					event_handler* payload_handler) {
		//std::cout << "src_outport: " << std::to_string(src_outport) << " and dst_inport: " << std::to_string(dst_inport) << std::endl;
		//outport_handler_[src_outport] = payload_handler;
		std::cout << "src_outport: " << std::to_string(src_outport) << " and dst_inport: " << std::to_string(dst_inport) << std::endl;
 		outport_handler_[0] = payload_handler;
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

	bool flexfly_optical_switch::outport_connected(int outport) const {
		for (int i = 0; i < num_ports_; i++) {
			if (inout_connection_[i] == outport) {
				return true;
			}
		}
		return false;
	}
}
}