#include "flexfly_optical_switch.h"
#include "flexfly_events.h"
#include <sstmac/hardware/common/connection.h>
#include <iostream>
#include <stdio.h>
#include <sstmac/hardware/pisces/pisces.h>
#include <sprockit/util.h>

#include <sstmac/software/launch/launch_event.h>
#include <sstmac/hardware/network/network_message.h>
//#include <sstmac/hardware/network/network_message.h>

#define DONT_CARE 1000


namespace sstmac {
namespace hw {
	flexfly_optical_switch::flexfly_optical_switch(sprockit::sim_parameters* params,
    												uint64_t id,
    												event_manager* mgr) : optical_switch(params, id, mgr) {
		//std::cout << "FLEXFLY OPTICAL SWITCH CONSTRUCTOR" << std::endl;
		
		my_addr_ = params->get_int_param("id");
		//std::cout << "FLEXFLY_OPTICAL_SWITCH" << std::endl;
		//std::printf("This address of this switch is: %d with pointer: %p\n", this->my_addr_, this);
		num_ports_ = params->get_int_param("optical_switch_radix");
		std::cout << "Number of ports in optical switch is: " <<std::to_string(num_ports_) << std::endl;
		num_ports_ = 100;
		inport_handler_.reserve(num_ports_);
		outport_handler_.reserve(num_ports_);
		inout_connection_ = new int[num_ports_];
		for (int i = 0; i < num_ports_; i++) {
			inout_connection_[i] = i;
		}
		top_ = topology::static_topology(params);
		init_links(params); // this has to be called upon class initialization
	};

	flexfly_optical_switch::~flexfly_optical_switch() {
		//std::printf("FLEXFLY_OPTICAL_SWITCH DECONSTRUCTOR pointer: %p with id: %d\n", this, this->my_addr_);
		delete [] inout_connection_;
	}

	void flexfly_optical_switch::setup() {
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
		
 		inport_connections_[dst_inport] = src_outport;
 		int input_port = dst_inport;
 		inport_handler_[input_port] = credit_handler;
		return;
		
	};

	void flexfly_optical_switch::connect_output(sprockit::sim_parameters* params, 
                              					int src_outport, 
                              					int dst_inport,
                              					event_handler* payload_handler) {
		//std::cout << "src_outport: " << std::to_string(src_outport) << " and dst_inport: " << std::to_string(dst_inport) << std::endl;
		//std::cout << "src_outport: " << std::to_string(src_outport) << " and dst_inport: " << std::to_string(dst_inport) << std::endl;
 		
 		outport_connections_[src_outport] = dst_inport;
 		int output_port = src_outport;
 		outport_handler_[output_port] = payload_handler;
		return;
	};

	link_handler* flexfly_optical_switch::payload_handler(int port) const {
		//std::printf("payload handler being called on switch with address: %p and addr is %d\n", this, this->my_addr_);
		std::cout << "OPTICAL_SWITCHL: PAYLOAD HANDLER" << std::endl;
		return new_link_handler(this, &flexfly_optical_switch::recv_payload);
	};

	link_handler* flexfly_optical_switch::credit_handler(int port) const {
		//std::printf("credit handler being called on switch with address: %p and addr is %d\n", this, this->my_addr_);
		std::cout << "OPTICAL_SWITCHL: CREDIT HANDLER" << std::endl;
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
		// figure out which link handler this is supposed to 
		std::cout << "RECV PAYLOAD ON AN OPTICAL SWITCH YABADABA optical swid: " << std::to_string(my_addr_) << std::endl;
		/*
		flexfly_payload_event* fev = dynamic_cast<flexfly_payload_event*>(ev);
		if (fev == nullptr) {
			return;
		}
		int switch_inport = fev->dst_inport();
		int switch_outport = inout_connection_[switch_inport];
		event_handler* handler = outport_handler_[switch_outport];
		int dst_inport = outport_connections_[switch_outport];
		*/
		if (!ev) {
			return;
		}
		message* msg = safe_cast(message, ev);
		//pisces_payload* msg = safe_cast(pisces_payload, ev);
		//int switch_inport = msg->inport();
		//int switch_outport = inout_connection_[switch_inport];
		//event_handler* handler = outport_handler_[switch_outport];
		node_id dst_id = msg->toaddr();
		node_id src_id = msg->fromaddr();
		switch_id dst_swid = top_->node_to_logp_switch(dst_id);
		int outport = 0;
		if (dst_id == 1) {
			outport = 1;
		}
		send_to_link(outport_handler_[outport],ev);
		//send_to_link(handler, msg);
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
			}	
		}
	}
}
}