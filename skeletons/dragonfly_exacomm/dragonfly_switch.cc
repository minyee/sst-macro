#include "dragonfly_switch.h"
#include <sstmac/hardware/common/connection.h>
#include <sprockit/sim_parameters.h>
#include <sprockit/util.h>
#include <sprockit/factories/factory.h>
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
		switches_per_group_ = params->get_int_param("switches_per_group");
		//num_groups_ = params->get_int_param("groups");
		switch_radix_ = params->get_int_param("switch_radix");
		nodes_per_switch_ = params->get_int_param("nodes_per_switch");
		sprockit::sim_parameters* link_params = params->get_namespace("link");
		electrical_link_bw_ = link_params->get_bandwidth_param("electrical_link_bandwidth");
		optical_link_bw_ = link_params->get_bandwidth_param("electrical_link_bandwidth");
		inv_optical_link_bw_ = 1 / optical_link_bw_;
		inv_electrical_link_bw_ = 1 / electrical_link_bw_;
		send_payload_latency_ = link_params->get_time_param("send_latency");
		send_credit_latency_ = link_params->get_time_param("credit_latency");
		credits_nodal_ = new int[nodes_per_switch_]; 
		credits_switch_ = new int[switch_radix_]; 
		std::memset(credits_nodal_, 0, sizeof(int) * nodes_per_switch_);
		std::memset(credits_switch_, 0, sizeof(int) * switch_radix_);
		// Initiate the outport handlers and their corresponding outport switches
		switch_inport_handlers_.resize(switch_radix_);
		switch_outport_handlers_.resize(switch_radix_);
		nodal_inport_handlers_.resize(nodes_per_switch_);
		nodal_outport_handlers_.resize(nodes_per_switch_);
		dtop_ = dynamic_cast<exacomm_dragonfly_topology *>(topology::static_topology(params));
		init_links(params);
	}

	dragonfly_switch::~dragonfly_switch() {
		if (credits_switch_ != nullptr)
			delete [] credits_nodal_;
		if (credits_nodal_ != nullptr) 
			delete [] credits_switch_;
	}

	int dragonfly_switch::queue_length(int port) const {
		return 0;
	}

	void dragonfly_switch::connect_input(sprockit::sim_parameters* params, 
                              						int src_outport, 
                              						int dst_inport,
                              						event_handler* credit_handler) {
		if (dst_inport < switch_radix_) {
			switch_inport_handlers_[dst_inport] = credit_handler;
		} else if (dst_inport <= switch_radix_ + nodes_per_switch_ - 1) {
			nodal_inport_handlers_[dst_inport - switch_radix_] = credit_handler;
		} else {
			spkt_abort_printf("Invalid connect inport for switch with id: %d", my_addr_);
		}
	}

	void dragonfly_switch::connect_output(sprockit::sim_parameters* params, 
                              						int src_outport, 
                              						int dst_inport,
                              						event_handler* payload_handler) {
		if (src_outport < switch_radix_) {
			switch_outport_handlers_[src_outport] = payload_handler;
		} else if (src_outport <= switch_radix_ + nodes_per_switch_ - 1) {
			nodal_outport_handlers_[src_outport - switch_radix_] = payload_handler;
		} else {
			spkt_abort_printf("Invalid connect outport for switch with id: %d", my_addr_);
		}
	}

	link_handler* dragonfly_switch::credit_handler(int port) const {
		if (port < switch_radix_) {
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
		int dst_switch = dtop_->node_to_switch(dst);
		int dst_group = dtop_->group_from_swid(dst_switch);
		int outport;
		timestamp actual_delay = send_payload_latency_ + (8 * msg->num_bytes()) * inv_electrical_link_bw_;
		//timestamp optical_delay = send_payload_latency_ + (8 * msg->num_bytes()) * inv_optical_link_bw_;
		event_handler* eh; 
		if (dst_switch == my_addr_) {
			outport = dst % nodes_per_switch_;
			eh = nodal_outport_handlers_[outport];
			//send_delayed_to_link(electrical_delay, nodal_outport_handlers_[outport], ev);
		} else {
			routable::path pth;
			dtop_->minimal_route_to_switch(my_addr_, dst_switch, pth);
			outport = pth.outport();
			eh = switch_outport_handlers_[outport];
			if (dst_group != dtop_->group_from_swid(my_addr_)) {
				actual_delay = send_payload_latency_ + (8 * msg->num_bytes()) * inv_optical_link_bw_;	
			} 
		}
		send_delayed_to_link(actual_delay, eh, ev);
		return;
	}

	/**
	 * Receiving a packet or message from a node, not from a switch.
	 * This can be said to be an injection packet. Must be routed through
	 * the network 
	 **/
	void dragonfly_switch::recv_nodal_payload(event* ev) {
		pisces_default_packet* msg = safe_cast(pisces_default_packet, ev);
		int src = msg->fromaddr();
		int dst = msg->toaddr();
		int src_switch = dtop_->node_to_switch(src);
		int dst_switch = dtop_->node_to_switch(dst);
		int src_group = dtop_->group_from_swid(src_switch);
		int dst_group = dtop_->group_from_swid(dst_switch);
		
		timestamp actual_delay = send_payload_latency_ + (8 * msg->num_bytes()) * inv_electrical_link_bw_;
		int outport; 
		event_handler* eh; 
		pisces_credit* cred = new pisces_credit(0, 0, 100000);
		send_delayed_to_link(send_credit_latency_, nodal_inport_handlers_[src % nodes_per_switch_], cred);
		if (my_addr_ == dst_switch) {
			outport = dst % nodes_per_switch_;
			eh = nodal_outport_handlers_[outport];
		} else {
			routable::path pth;
			dtop_->minimal_route_to_switch(my_addr_, dst_switch, pth); // CONTINUE HERE, THIS IS WHERE IT FAILS
			outport = pth.outport();
			eh = switch_outport_handlers_[outport];
			if (dst_group != src_group) {
				actual_delay = send_payload_latency_ + (8 * msg->num_bytes()) * inv_optical_link_bw_;	
			} 
		} 
		send_delayed_to_link(actual_delay, eh, ev);

	};

	void dragonfly_switch::recv_nodal_credit(event* ev) {
		pisces_credit* cred = safe_cast(pisces_credit, ev);
		credits_nodal_[cred->port()] += cred->num_credits();
		return;
	};

	void dragonfly_switch::recv_credit(event* ev) {
		pisces_credit* cred = safe_cast(pisces_credit, ev);
		credits_switch_[cred->port()] += cred->num_credits();
		return;
	};

	//void dragonfly_switch::send_credit() {

	//}
}
}