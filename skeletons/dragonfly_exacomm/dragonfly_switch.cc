#include "dragonfly_switch.h"
#include <sstmac/hardware/common/connection.h>
#include <sprockit/sim_parameters.h>
#include <sprockit/util.h>
#include <sstmac/hardware/switch/network_switch.h>
#include <sprockit/factories/factory.h>
#include <sstmac/hardware/router/minimal_routing.h>
#include <sstmac/hardware/pisces/pisces.h>
#include <sstmac/hardware/network/network_message.h>

namespace sstmac {
namespace hw {

	dragonfly_switch::dragonfly_switch(sprockit::sim_parameters* params,
    														uint64_t id,
    														event_manager* mgr,
    														device_id::type_t ty) : 
																network_switch(params, 
																				id,
																				mgr, 
																				device_id::logp_overlay) {
		my_addr_ = params->get_int_param("id");
		radix_ = params->get_int_param("num_ports");
		switches_per_group_ = params->get_int_param("switches_per_group");
		num_groups_ = params->get_int_param("num_groups");
		num_optical_links_ = params->get_int_param("num_optical_links");
		nodes_per_switch_ = params->get_int_param("nodes_per_switch");
		
		// Initiate the outport handlers and their corresponding outport switches
		inport_handlers_.resize(radix_);
		outport_handlers_.resize(radix_);
		inport_switch_.resize(radix_);
		outport_switch_.resize(radix_);
		dtop_ = dynamic_cast<exacomm_dragonfly_topology *>(topology::static_topology(params));

		std::string outport_prefix = "outport";
		std::string inport_prefix = "inport";

		for (int i = 0; i < radix_; i++) {
			int target_switch = params->get_int_param(outport_prefix + std::to_string(i));
			int source_switch = params->get_int_param(inport_prefix + std::to_string(i));
			outport_switch_[i] = target_switch;
			inport_switch_[i] = source_switch;
			dtop_->set_connection(my_addr_, i, target_switch, false);
			dtop_->set_connection(my_addr_, i, source_switch, true);
		}
		init_links(params);
	}

	dragonfly_switch::~dragonfly_switch() {
	}

	int dragonfly_switch::queue_length(int port) const {
		return 0;
	}

	void dragonfly_switch::connect_input(sprockit::sim_parameters* params, 
                              						int src_outport, 
                              						int dst_inport,
                              						event_handler* credit_handler) {
		inport_handlers_[dst_inport] = credit_handler;
	}

	void dragonfly_switch::connect_output(sprockit::sim_parameters* params, 
                              						int src_outport, 
                              						int dst_inport,
                              						event_handler* payload_handler) {
		std::cout << "connect output is called on switch: " << std::to_string(my_addr_) << " with src_outport: " << std::to_string(src_outport) << " and dst_inport: " << std::to_string(dst_inport) << std::endl;
		outport_handlers_[src_outport] = payload_handler;
	}

	link_handler* dragonfly_switch::credit_handler(int port) const {
		if (port < switches_per_group_) {
			return new_link_handler(this, &dragonfly_switch::recv_credit);
		} else {
			return new_link_handler(this, &dragonfly_switch::recv_nodal_credit);
		}
	}

	link_handler* dragonfly_switch::payload_handler(int port) const {
		if (port < switches_per_group_) {
			return new_link_handler(this, &dragonfly_switch::recv_payload);
		} else {
			return new_link_handler(this, &dragonfly_switch::recv_nodal_payload);
		}
	}

	/**
	 * Receiving a packet or message from another switch, not from a node.
	 **/
	void dragonfly_switch::recv_payload(event* ev) { 
  		pisces_default_packet* msg = safe_cast(pisces_default_packet, ev);
  		int dst = msg->toaddr();
		int src = msg->fromaddr();
		int dst_switch = dtop_->node_to_switch(dst);
		int src_switch = dtop_->node_to_switch(src);
		int dst_group = dtop_->group_from_swid(dst_switch);
		int src_group = dtop_->group_from_swid(src_switch);
		if (dst_switch == my_addr_) {
			int offset = dst % nodes_per_switch_;
			send_to_link(outport_handlers_[offset + switches_per_group_], msg);
		} else if (dst_group == dtop_->group_from_swid(my_addr_)) {
			//port_to_switch();
			int outport = 0;
			for (outport = 0; outport < radix_; outport++) {
				if (outport_switch_[outport] == dst_switch) {
					send_to_link(outport_handlers_[outport], ev);
					return;
				}
			}
			//outport = dtop_->get_output_port(my_addr_, dst_switch);
			
		} else {
			assert(dst_group != dtop_->group_from_swid(my_addr_));
			send_to_link(outport_handlers_[switches_per_group_ - 1], msg);
		}
  		
	}

	/**
	 * Receiving a packet or message from a node, not from a switch.
	 * This can be said to be an injection packet. Must be routed through
	 * the network 
	 **/
	void dragonfly_switch::recv_nodal_payload(event* ev) {
		pisces_default_packet* msg = safe_cast(pisces_default_packet, ev);
		
		
		int dst = msg->toaddr();
		int src = msg->fromaddr();
		int dst_switch = dtop_->node_to_switch(dst);
		int src_switch = dtop_->node_to_switch(src);
		int dst_group = dtop_->group_from_swid(dst_switch);
		int src_group = dtop_->group_from_swid(src_switch);
		int port_index = num_groups_ * switches_per_group_ + num_optical_links_;;
		if (src_switch == dst_switch) {
			port_index += (dst % nodes_per_switch_);
		} else if (false) {
			
			for (auto swid : outport_switch_) {
				if (swid == 1) {
					
				}
			}
		}
		send_to_link(outport_handlers_[port_index], ev);
	};

	void dragonfly_switch::recv_nodal_credit(event* ev) {
		return;
	};

	void dragonfly_switch::recv_credit(event* ev) {
		return;
	};
}
}