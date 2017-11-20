#include "flexfly_electrical_switch.h"
//#include "flexfly_packet.h"
#include <sstmac/hardware/common/connection.h>
#include <sstmac/hardware/topology/topology.h>
#include <sprockit/sim_parameters.h>
#include <sprockit/util.h>
#include <sstmac/hardware/router/router_fwd.h>
#include <sstmac/hardware/switch/network_switch.h>
#include <sprockit/factories/factory.h>
#include <sstmac/hardware/router/minimal_routing.h>
#include <sstmac/hardware/pisces/pisces.h>
#include <sstmac/software/launch/launch_event.h>
#include <sstmac/hardware/network/network_message.h>

namespace sstmac {
namespace hw {

	flexfly_electrical_switch::flexfly_electrical_switch(sprockit::sim_parameters* params,
    														uint64_t id,
    														event_manager* mgr,
    														device_id::type_t ty) : 
																network_switch(params, 
																				id,
																				mgr, 
																				device_id::logp_overlay) {
		//static int my_int = 0;
		//std::cout << "my_int : " + std::to_string(my_int) << std::endl;
		
		//my_id_ = my_int;
		my_addr_ = params->get_int_param("id");
		std::cout << "Electrical switch constructor for addr: " << std::to_string(my_addr_) << std::endl;
		radix_ = params->get_int_param("total_radix");
		switches_per_group_ = params->get_int_param("switches_per_group");
		num_groups_ = params->get_int_param("num_groups");
		nodes_per_switch_ = radix_ - switches_per_group_;
		inport_handlers_.reserve(radix_);
		outport_handlers_.reserve(radix_);
		queue_length_ = new int[10];
		sprockit::sim_parameters* rtr_params = params->get_optional_namespace("router");
		rtr_params->add_param_override_recursive("id", int(my_addr_));
		router_ = router::factory::get_param("name", rtr_params, top_, this);
		ftop_ = safe_cast(flexfly_topology, top_);
		init_links(params);
		//my_int++;
	}

	flexfly_electrical_switch::~flexfly_electrical_switch() {
		delete [] queue_length_;
	}

	int flexfly_electrical_switch::queue_length(int port) const {
		return queue_length_[port];
	}

	void flexfly_electrical_switch::connect_input(sprockit::sim_parameters* params, 
                              						int src_outport, 
                              						int dst_inport,
                              						event_handler* credit_handler) {
		if (dst_inport < 0 || dst_inport >= radix_)
			spkt_abort_printf("Invalid inport %d in flexfly_electrical_switch::connect_input", dst_inport);
		inport_handlers_[dst_inport] = credit_handler;
	}

	void flexfly_electrical_switch::connect_output(sprockit::sim_parameters* params, 
                              						int src_outport, 
                              						int dst_inport,
                              						event_handler* payload_handler) {
		if (src_outport < 0 || src_outport >= radix_)
			spkt_abort_printf("Invalid inport %d in flexfly_electrical_switch::connect_output", src_outport);
		outport_handlers_[src_outport] = payload_handler;
	}

	link_handler* flexfly_electrical_switch::credit_handler(int port) const {
		if (port < switches_per_group_) {
			return new_link_handler(this, &flexfly_electrical_switch::recv_credit);
		} else {
			return new_link_handler(this, &flexfly_electrical_switch::recv_nodal_credit);
		}
	}

	link_handler* flexfly_electrical_switch::payload_handler(int port) const {
		if (port < switches_per_group_) {
			return new_link_handler(this, &flexfly_electrical_switch::recv_payload);
		} else {
			return new_link_handler(this, &flexfly_electrical_switch::recv_nodal_payload);
		}
	}

	/**
	 * Receiving a packet or message from another switch, not from a node.
	 **/
	void flexfly_electrical_switch::recv_payload(event* ev) {
		flexfly_packet* fpacket = safe_cast(flexfly_packet, ev);
		node_id dst = fpacket->get_pisces_packet()->toaddr();
  		node_id src = fpacket->get_pisces_packet()->fromaddr();
  		
  		std::cout << "received_payload?" << std::endl;
  		std::cout << "Electrical switch: " << std::to_string(my_addr_) << " received a packet" << std::endl;
  		std::cout << "From node: " << std::to_string(src) << " to node: " + std::to_string(dst) << std::endl;
  		// Case 1: route it to a connecting node
  		switch_id dst_swid = ftop_->node_to_switch(dst);
  		if (dst_swid == my_addr_) {
  			int outport = dst - (my_addr_ * nodes_per_switch_) + switches_per_group_;
  			send_to_link(outport_handlers_[outport], fpacket->get_pisces_packet());
  			return;
  		}
  		send_to_link(outport_handlers_[fpacket->next_outport()], fpacket);
	}

	/**
	 * Receiving a packet or message from a node, not from a switch.
	 * This can be said to be an injection packet. Must be routed through
	 * the network 
	 **/
	void flexfly_electrical_switch::recv_nodal_payload(event* ev) {
		// need to generate a new flexfly_packet here
		pisces_default_packet* msg = safe_cast(pisces_default_packet, ev);
		std::cout << "received_nodal_payload?" << std::endl;
		std::cout << "From node: " << std::to_string(msg->fromaddr()) << " to node: " + std::to_string(msg->toaddr()) << std::endl;
		int dst = msg->toaddr();
		int src = msg->fromaddr();
		int dst_switch = ftop_->node_to_switch(dst);
		int src_switch = ftop_->node_to_switch(src);
		if (src_switch == dst_switch) {
			int outport = dst - (my_addr_ * nodes_per_switch_) + switches_per_group_;
			std::cout << "my_id_ is: " << std::to_string(my_addr_) << std::endl;
			std::cout << "outport is: " << std::to_string(outport) << std::endl;
			send_to_link(outport_handlers_[outport], ev);
			return; 
		}

		flexfly_packet* fpacket = new flexfly_packet(msg);
		ftop_->route_minimal(my_addr_, ftop_->node_to_switch(msg->toaddr()), fpacket);
		int next_port = fpacket->next_outport();
		std::cout << "Next port is: " + std::to_string(next_port) << std::endl;
		//assert(outport_handlers_[]);
		send_to_link(outport_handlers_[next_port], fpacket);
		/*
		if (ftop_->node_to_switch(msg->toaddr()) == my_addr_) {
			send_to_link(outport_handlers_[1], ev);
		} else {
			send_to_link(outport_handlers_[0], ev);
		}
		*/
	};

	void flexfly_electrical_switch::recv_nodal_credit(event* ev) {
		return;
	};

	void flexfly_electrical_switch::recv_credit(event* ev) {
		return;
	};

	int flexfly_electrical_switch::node_to_port(int node) const {
		int port = -1;
		if (node < my_addr_ * switches_per_group_ * num_groups_) {

		}
		return -1;
	}

	/**
	 * Has to be called when received a packet and the target node is
	 * connected to current switch
	 **/
	void flexfly_electrical_switch::send_packet_to_node(event* ev, int node_id) {
		int port_id = node_to_port(node_id);
		send_to_link(outport_handlers_[port_id], ev);
	}
}
}