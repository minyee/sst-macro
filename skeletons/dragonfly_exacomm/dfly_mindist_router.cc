#include "dfly_mindist_router.h"


namespace sstmac {
namespace hw {

dfly_mindist_router::dfly_mindist_router(sprockit::sim_parameters* params, topology* top, network_switch* netsw) : {
	my_addr_ = netsw->switch_id();
};

dfly_mindist_router::~dfly_mindist_router() {
	
}

void dfly_mindist_router::route_to_switch(switch_id sw_addr, routable::path& path) const {
	my_addr_ = ; // how the fuck do I find my_addr?
	top_->minimal_route_to_switch(my_addr_, sw_addr, path);

	return;
};

}	
}