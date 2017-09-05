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
		inport_handlers_.reserve(radix_);
		outport_handlers_.reserve(radix_);
		
		queue_length_ = new int[10];
		sprockit::sim_parameters* rtr_params = params->get_optional_namespace("router");
		rtr_params->add_param_override_recursive("id", int(my_addr_));
		router_ = router::factory::get_param("name", rtr_params, top_, this);
		std::cout << "FLEXFLY_ELECTRICAL_SWITCH CONSTRUCTOR for swid: " << std::to_string(my_addr_) << std::endl;
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
		inport_handlers_[dst_inport] = credit_handler;
	}

	void flexfly_electrical_switch::connect_output(sprockit::sim_parameters* params, 
                              						int src_outport, 
                              						int dst_inport,
                              						event_handler* payload_handler) {
		outport_handlers_[src_outport] = payload_handler;
	}

	link_handler* flexfly_electrical_switch::credit_handler(int port) const {
		//std::cout << "CREDIT HANDLERRRRR" << std::endl;
		//return nullptr;
		//if (port == radix_ - 1) 
		//	return new_link_handler(this, &flexfly_electrical_switch::recv_nodal_msg);
		return new_link_handler(this, &flexfly_electrical_switch::recv_credit);
	}

	link_handler* flexfly_electrical_switch::payload_handler(int port) const {
		//std::cout << "PAYYLOADD HANDLER" << std::endl;
		return new_link_handler(this, &flexfly_electrical_switch::recv_payload);
	}

	void flexfly_electrical_switch::bcast_local_message(message* msg, node_id src) {
		
  		sw::start_app_event* lev = safe_cast(sw::start_app_event, msg);
  		sw::task_mapping::ptr mapping = lev->mapping();
  		int num_ranks = mapping->num_ranks();
  		std::cout << "num_ranks is: " << std::to_string(num_ranks) << std::endl;
  		
  		for (int i=0; i < num_ranks; ++i){
    		node_id dst_node = mapping->rank_to_node(i);
    		std::cout << "Rank : " << std::to_string(i) << " in Node: " << std::to_string(dst_node) << std::endl;
    		std::cout << "dst_node : " << std::to_string(i) << " my_addr_: " << std::to_string(my_addr_) << std::endl;
    		if (dst_node != my_addr_) {
    			sw::start_app_event* new_lev = lev->clone(i,src, dst_node);
    			std::printf("pointer to outport handler 0 : %p\n",outport_handlers_[0]);
    			send_to_link(outport_handlers_[0], new_lev);
    			std::cout << "here?" << std::endl;
    		} else {

    		}
    		//auto dst_nic = nics_[dst_node];
    		//bool local_dst = dst_nic;
    		//bool local_src = nics_[src];
    		/*
    		if (local_dst) {
      			sw::start_app_event* new_lev = lev->clone(i, src, dst_node);
      			int num_hops = top_->num_hops_to_node(src, dst_node);
      			timestamp delay = num_hops * hop_latency_;
      			if (local_src){ //have to accumulate inj latency here
        			delay += dbl_inj_lat_; //factor of 2 for in-out
      			}
      			send_delayed_to_link(delay, dst_nic, new_lev);
    		}
    		*/
  		}
  		
  		//this one not needed anymore
  		delete lev;
  		
	}

	void flexfly_electrical_switch::recv_payload(event* ev) {
		//flexfly_payload_event* fev = dynamic_cast<flexfly_payload_event*>(ev);
		std::cout << "RECEIVED A RECV_PAYLOAD at switch id: " << std::to_string(my_addr_) << " and argument id: " << std::to_string(my_id_) << std::endl;
		pisces_payload* msg1 = safe_cast(pisces_payload, ev);
		message* msg = safe_cast(message, ev);
		if (!msg) {
			return;
		}
		node_id dst = msg->toaddr();
  		node_id src = msg->fromaddr();
  		//if (dst == my_addr_) {
  			//send_to_link(outport_handlers_[radix_ - 1], ev);
  			//return;
  		//}

  		if (msg->is_bcast()) {
  			sw::start_app_event* lev = safe_cast(sw::start_app_event, msg);
  			sw::task_mapping::ptr mapping = lev->mapping();
  			int num_ranks = mapping->num_ranks();
  			std::cout << "IT IS A BCAST MESSAGE" << std::endl;
  			for (int i = 0; i < num_ranks; i++) {
  				node_id dst_node = mapping->rank_to_node(i);
  				if (dst_node == my_addr_)
  					bcast_local_message(msg, my_addr_);	
  				else {
  					std::cout << "guard1" << std::endl;
  					std::printf("pointer to outport handler 0 : %p\n",outport_handlers_[0]);
  					send_to_link(outport_handlers_[0], ev);
  					std::cout << "guard2" << std::endl;
  				}
  			}
  			
  			//return;
  		}
		//pisces_payload* msg = dynamic_cast<pisces_payload*>(ev);
		
		//pisces_payload* msg = safe_cast(pisces_payload, ev);
		std::cout << "The to_addr is: " << std::to_string(msg->toaddr()) << std::endl;
		std::cout << "The from_addr is: " << std::to_string(msg->fromaddr()) << std::endl;
		//std::cout << "The num_bytes is: " << std::to_string(msg->num_bytes()) << std::endl;
		//uint64_t flow_id = msg->flow_id();
		//node_id dst = msg->toaddr();
  		//node_id src = msg->fromaddr();
  		//std::cout << "dst nodeid: " << std::to_string(dst) << " and src nodeid: " << std::to_string(src) << std::endl;
		//send_to_link(outport_handlers_[0], msg);
		//fev->

	}

	void flexfly_electrical_switch::recv_nodal_msg(event* ev) {
		std::cout << "RECEIVED A NODAL MESSAGE" << std::endl;
	};

	void flexfly_electrical_switch::recv_credit(event* ev) {
		std::cout << "RECEIVED A NODAL RECV_CREDIT" << std::endl;
		return;
	};

	flexfly_payload_event* flexfly_electrical_switch::random_forward(switch_id src_id, int src_outport) const {
		int dst_inport = 1;
		switch_id  dst_id = 1;
		return new flexfly_payload_event(src_id, src_outport, dst_id, dst_inport);
	};
}
}