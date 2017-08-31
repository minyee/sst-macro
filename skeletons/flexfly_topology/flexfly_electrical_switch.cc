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
		my_addr_ = params->get_int_param("id");
		radix_ = params->get_int_param("total_radix");
		inport_handlers_.reserve(radix_);
		outport_handlers_.reserve(radix_);
		init_links(params);
		queue_length_ = new int[10];
		sprockit::sim_parameters* rtr_params = params->get_optional_namespace("router");
		rtr_params->add_param_override_recursive("id", int(my_addr_));
		router_ = router::factory::get_param("name", rtr_params, top_, this);
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
		if (port == radix_ - 1) 
			return new_link_handler(this, &flexfly_electrical_switch::recv_nodal_msg);
		return new_link_handler(this, &flexfly_electrical_switch::recv_credit);
	}

	link_handler* flexfly_electrical_switch::payload_handler(int port) const {
		//std::cout << "PAYYLOADD HANDLER" << std::endl;
		return new_link_handler(this, &flexfly_electrical_switch::recv_payload);
	}

	void flexfly_electrical_switch::recv_payload(event* ev) {
		auto* fev = dynamic_cast<flexfly_payload_event*>(ev);
		std::cout << "RECEIVED A NODAL MESSAGE" << std::endl;
		if (fev == nullptr) {
			std::cout << "SHIT SHIT SHIT" << std::endl;
			return;
		}
		send_to_link(outport_handlers_[1], random_forward(1,1));
		//fev->

	}

	void flexfly_electrical_switch::recv_nodal_msg(event* ev) {
		std::cout << "RECEIVED A NODAL MESSAGE" << std::endl;
	};

	void flexfly_electrical_switch::recv_credit(event* ev) {
		std::cout << "RECEIVED A NODAL MESSAGE" << std::endl;
		return;
	};

	flexfly_payload_event* flexfly_electrical_switch::random_forward(switch_id src_id, int src_outport) const {
		int dst_inport = 1;
		switch_id  dst_id = 1;
		return new flexfly_payload_event(src_id, src_outport, dst_id, dst_inport);
	};
}
}