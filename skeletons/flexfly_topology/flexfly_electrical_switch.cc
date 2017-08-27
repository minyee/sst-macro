#include "flexfly_electrical_switch.h"
#include <sstmac/hardware/common/connection.h>

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
		//std::cout << "FLEXFLY ELECTRICAL SWITCH CONSTRUCTOR" << std::endl;
		my_addr_ = params->get_int_param("id");
		std::cout << "FLEXFLY ELECTRICAL SWITCH" << std::endl;
		std::cout << "The address of this electrical switch is: "<< std::to_string(my_addr_) << std::endl;
		init_links(params);
		queue_length_ = new int[10];
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

	}

	void flexfly_electrical_switch::connect_output(sprockit::sim_parameters* params, 
                              						int src_outport, 
                              						int dst_inport,
                              						event_handler* credit_handler) {
		
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