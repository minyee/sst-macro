#include "flexfly_electrical_switch.h"
#include <sstmac/hardware/common/connection.h>
#include <sstmac/hardware/topology/topology.h>
#include <sprockit/sim_parameters.h>
#include <sstmac/hardware/nic/nic.h>
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
		radix_ = params->get_int_param("radix");
		// std::cout << "FLEXFLY ELECTRICAL SWITCH" << std::endl;
		std::cout << "The address of this electrical switch is: "<< std::to_string(my_addr_) << std::endl;
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
		inport_handlers[dst_inport] = credit_handler;
	}

	void flexfly_electrical_switch::connect_output(sprockit::sim_parameters* params, 
                              						int src_outport, 
                              						int dst_inport,
                              						event_handler* payload_handler) {
		outport_handlers[src_outport] = payload_handler;
	}

	link_handler* flexfly_electrical_switch::credit_handler(int port) const {
		return new_link_handler(this, &flexfly_electrical_switch::recv_credit);
	}

	link_handler* flexfly_electrical_switch::payload_handler(int port) const {
		return new_link_handler(this, &flexfly_electrical_switch::recv_payload);
	}

	void flexfly_electrical_switch::recv_payload(event* ev) {
		return;
	}

	void flexfly_electrical_switch::recv_credit(event* ev) {
		return;
	}
}
}