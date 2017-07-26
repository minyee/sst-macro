#include <sstmac/hardware/switch/network_switch.h>
#include <sstmac/hardware/topology/topology.h>
#include <vector> 

namespace sstmac {
namespace hw {

typedef port_id;

class flexfly_electrical_switch : public network_switch {
  FactoryRegister("flexfly_electrical_switch", network_switch, flexfly_electrical_switch);
 
 protected:

 flexfly_electrical_switch(sprockit::sim_parameters* params,
    uint64_t id,
    event_manager* mgr,
    device_id::type_t ty = device_id::router) : network_switch(params, id, mgr, ty);
 
 ~flexfly_electrical_switch();  

 public:
 
 void connected_outports(std::vector<connection*>& conn_vector) {
 	for (connection* conn : conn_vector) {

 	}
 }

 private:
 void (std::vector<>)	
 //std::unordered_map<port_>
 // need a data structure that links a port id to a switch_id or 
 std::unordered_map<port_id,std::vector<connection>>;
}
 
 
}
}