#include <sstmac/hardware/switch/flexfly_optical_switch.h>
#include <sstmac/hardware/common/connection.h>
#include <iostream>


namespace sstmac {
namespace hw {
	void flexfly_optical_switch::flexfly_optical_switch() {
		init_connection(params); // this has to be called upon class initialization
		std


	}

	void flexfly_optical_switch::init(unsigned int phase) {
		init(phase);
		//connection::init(phase);
		return; 
	}
}
}