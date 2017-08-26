#include <sstmac/hardware/switch/network_switch.h>
#include <sstmac/hardware/topology/topology.h>
#include <vector> 

namespace sstmac {
namespace hw {

//typedef int port_id;

class flexfly_electrical_switch : public network_switch {
public:

RegisterComponent("flexfly_electrical_switch | flexfly_electrical", network_switch, flexfly_electrical_switch,
         			"macro", COMPONENT_CATEGORY_NETWORK,
         			"An electrical switch which has electrical components to it which is used in the Flexfly project");
 
flexfly_electrical_switch(sprockit::sim_parameters* params,
							uint64_t id,
							event_manager* mgr,
							device_id::type_t ty = device_id::router);// : network_switch(params, id, mgr, ty);
 
~flexfly_electrical_switch();

virtual std::string to_string() const override {
	return "Flexfly Electrical Switch for the Flexfly project";
};

/**
 * @brief connect_output
 * @param params
 * @param src_outport
 * @param dst_inport
 * @param payload_handler
 */
virtual void connect_output(sprockit::sim_parameters* params, 
                                int src_outport, 
                                int dst_inport, 
                                event_handler* payload_handler) override;


/**
 * @brief connect_input
 * @param params
 * @param src_outport
 * @param dst_inport
 * @param credit_handler Can be null, if no credits are ever sent
 */
virtual void connect_input(sprockit::sim_parameters* params, 
                              int src_outport, 
                              int dst_inport,
                              event_handler* credit_handler) override;

/**
 * @brief credit_handler
 * @param port
 * @return Can be null, if no credits are ever to be received
 */
virtual link_handler* credit_handler(int port) const override;

/**
 * @brief payload_handler
 * @param port
 *  @return
 */
virtual link_handler* payload_handler(int port) const override;

virtual int queue_length(int port) const override;

protected:

void recv_payload(event* ev);

void recv_credit(event* ev);

private:
int* queue_length_;
	
int port_cnt_;

 //void (std::vector<>)
 //std::unordered_map<port_>
	//need a data structure that links a port id to a switch_id or 
 //std::unordered_map<port_id,std::vector<connection>>;
};
 
}
}