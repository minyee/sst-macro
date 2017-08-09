#include "flexfly_optical_switch.h"
#include <sstmac/hardware/common/connection.h>
#include <iostream>


namespace sstmac {
namespace hw {
	flexfly_optical_switch::flexfly_optical_switch(sprockit::sim_parameters* params,
    uint64_t id,
    event_manager* mgr,
    device_id::type_t ty) : sstmac::hw::connectable_component(params, id, device_id(params->get_int_param("id"), ty), mgr) {
		//init_connection(params); // this has to be called upon class initialization
		//std


	}

	void flexfly_optical_switch::init(unsigned int phase) {
		init(phase);
		//connection::init(phase);
		return; 
	}
}
}