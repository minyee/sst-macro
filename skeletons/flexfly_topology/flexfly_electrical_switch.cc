#include "flexfly_electrical_switch.h"
//#include "flexfly_events.h"
#include <sstmac/hardware/common/connection.h>
#include <sstmac/hardware/topology/topology.h>
#include <sprockit/sim_parameters.h>
//#include <sstmac/hardware/nic/nic.h>
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
		my_id_ = id;
		my_addr_ = params->get_int_param("id");
		radix_ = params->get_int_param("total_radix");
		switches_per_group_ = params->get_int_param("switches_per_group");
		nodes_per_switch_ = radix_ - switches_per_group_;
		inport_handlers_.reserve(radix_);
		outport_handlers_.reserve(radix_);
		queue_length_ = new int[10];
		sprockit::sim_parameters* rtr_params = params->get_optional_namespace("router");
		rtr_params->add_param_override_recursive("id", int(my_addr_));
		router_ = router::factory::get_param("name", rtr_params, top_, this);
		ftop_ = safe_cast(flexfly_topology, top_);
		init_links(params);
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
		//std::cout << "RECEIVED A RECV_PAYLOAD at switch id: " << std::to_string(my_addr_) << " and argument id: " << std::to_string(my_id_) << std::endl;
		pisces_default_packet* msg = safe_cast(pisces_default_packet, ev);
		node_id dst = msg->toaddr();
  		node_id src = msg->fromaddr();
  		
  		std::cout << "Electrical switch: " << std::to_string(my_addr_) << " received a packet" << std::endl;
  		std::cout << "This packet has dst : " << std::to_string(dst) << " and src: " << std::to_string(src) << std::endl;
  		// Case 1: route it to a connecting node
  		flexfly_topology* ftop = safe_cast(flexfly_topology, top_);
  		switch_id dst_swid = ftop->node_to_switch(dst);
  		int dst_group = ftop->group_from_swid(dst_swid);
  		int my_group = ftop->group_from_swid(my_addr_);
  		router_->route(msg);

  		if (my_addr_ == ftop->node_to_switch(dst)) {
  			int port_num = dst - (my_addr_ * nodes_per_switch_) + switches_per_group_;
  			std::cout << "dst : " << std::to_string(dst) << " and my_addr is: " << std::to_string(my_addr_) << std::endl;
  			std::cout << "Port num is: " << std::to_string(port_num) << std::endl;
  			send_to_link(outport_handlers_[port_num], ev);
  		} else if (my_group == dst_group) { // thisi s not the destination switch bu the destination switch is in the same group
  			

  			//send_to_link(outport_handlers_[0], ev);	
  		}
	}

	/**
	 * Receiving a packet or message from a node, not from a switch.
	 * This can be said to be an injection packet
	 **/
	void flexfly_electrical_switch::recv_nodal_payload(event* ev) {
		std::cout << "received nodal message at switch " << std::to_string(my_addr_) << std::endl;
		pisces_default_packet* msg = safe_cast(pisces_default_packet, ev);
		if (ftop_->node_to_switch(msg->toaddr()) == my_addr_) {
			send_to_link(outport_handlers_[1], ev);
		} else {
			send_to_link(outport_handlers_[0], ev);
		}
	};

	void flexfly_electrical_switch::recv_nodal_credit(event* ev) {
		return;
	};

	void flexfly_electrical_switch::recv_credit(event* ev) {
		return;
	};
}
}