#include "flexfly_optical_switch.h"
#include <sstmac/hardware/common/connection.h>
#include <iostream>
#include <stdio.h>

namespace sstmac {
namespace hw {
	flexfly_optical_switch::flexfly_optical_switch(sprockit::sim_parameters* params,
    												uint64_t id,
    												event_manager* mgr) : optical_switch(params, id, mgr) {
		//std::cout << "FLEXFLY OPTICAL SWITCH CONSTRUCTOR" << std::endl;
		init_links(params); // this has to be called upon class initialization
		my_addr_ = params->get_int_param("id");
		std::cout << "FLEXFLY_OPTICAL_SWITCH" << std::endl;
		std::cout << "This address of this switch is: " << std::to_string(my_addr_) << std::endl;
		num_ports_ = params->get_int_param("optical_switch_radix");
		inout_connection_ = new int[num_ports_];
		for (int i = 0; i < num_ports_; i++) {
			inout_connection_[i] = i;
		}
	};

	flexfly_optical_switch::~flexfly_optical_switch() {
		//std::cout << "FLEXFLY_OPTICAL_SWITCH DECONSTRUCTOR" << std::endl;
		delete [] inout_connection_;
	}

	void flexfly_optical_switch::setup() {
		//init(phase);
		//connection::init(phase);makes
		return; 
	};

	/**
	 * NOTE: This method is only called once, and it hooks the input port to a handler 
	 * 			so that when the switch receives an 
	 */
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
		std::cout << "src_outport: " << std::to_string(src_outport) << " and dst_inport: " << std::to_string(dst_inport) << std::endl;
 		outport_handler_[0] = payload_handler;
		return;
	};

	link_handler* flexfly_optical_switch::payload_handler(int port) const {
		std::printf("payload handler being called on switch with address: %p and addr is %d\n", this, my_addr_);
		return new_link_handler(this, &flexfly_optical_switch::recv_payload);
	};

	link_handler* flexfly_optical_switch::credit_handler(int port) const {
		std::printf("credit handler being called on switch with address: %p and addr is %d\n", this, my_addr_);
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

	bool flexfly_optical_switch::setup_inout_connection(int inport, int outport) {
		// check if the current outport is already connected to any of the inports
		if (outport_connected(outport)) {
			return false;
		}
		// schedule some sort of delay?
		teardown_outport_connection(outport);
		inout_connection_[inport] = outport; 
		return true;
	}

	void flexfly_optical_switch::teardown_outport_connection(int outport) {
		for (int i = 0; i < num_ports_; i++) {
			if (inout_connection_[i] == outport) {
				inout_connection_[i] = -1;
				// schedule some sort of delay maybe?
			}	
		}
	}
}
}