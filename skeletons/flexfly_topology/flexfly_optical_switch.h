/**
 * Author: Min Yee Teh
 *
 */
#include <sprockit/factories/factory.h>
#include <sprockit/debug.h>

//#include <sstmac/hardware/common/connection.h> // have to import connection because it contains the prototype for connectable_component
                                              // which flexfly_optical_switch inherits
#include <sstmac/hardware/router/router_fwd.h>
//#include <sstmac/hardware/topology/topology_fwd.h>
#include <sstmac/hardware/topology/topology.h>
//#include <sstmac/common/event_scheduler.h>
#include <sprockit/sim_parameters_fwd.h>

#if SSTMAC_INTEGRATED_SST_CORE
#include <sstmac/sst_core/integrated_component.h>
#endif

#include <vector>
//#include <sst/core/event.h>

//DeclareDebugSlot(network_switch)
#define switch_debug(...) \
  debug_printf(sprockit::dbg::network_switch, "Switch %d: %s", \
    int(addr()), sprockit::printf(__VA_ARGS__).c_str())
//using namespace sstmac;
//using namespace sstmac::hw;
//using namespace SST;

//template <class T, class Fxn>
//sstmac::link_handler*
//new_link_handler(const T* t, Fxn fxn){
// return new_handler<T,Fxn>(const_cast<T*>(t), fxn);
//}
//template <class T, class Fxn>
//sstmac::link_handler* new_link_handler(const T* t, Fxn fxn){
  //return new SST::Event::Handler<T>(const_cast<T*>(t), fxn);
//}


namespace sstmac {
namespace hw {

class optical_switch : public connectable_component {
  public:
  DeclareFactory(optical_switch, uint64_t, event_manager*);

  optical_switch(sprockit::sim_parameters* params, uint64_t id, event_manager* mgr) :
    connectable_component(params,id,
      device_id(params->get_int_param("id"),device_id::router),mgr) {
    };
};  
/**
 * @brief The network_switch class
 * A class encapsulating a network switch that packets must traverse on the network.
 * The network switch performs both routing computations and congestion modeling.
 */
class flexfly_optical_switch :
  public optical_switch
{
  
  FactoryRegister("flexfly optical switch", optical_switch, flexfly_optical_switch, "This is flexfly optical switch");
 public:
  virtual ~flexfly_optical_switch();

  virtual std::string to_string() const override {
    return "Flexfly Optical Switch";
  };

  virtual void init(unsigned int phase) override;

  virtual void deadlock_check() override {return;};

  virtual void deadlock_check(event* ev) override {return;};

  virtual void setup() override; //needed for SST core compatibility

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
   * @return
   */
  virtual link_handler* payload_handler(int port) const override;// either if we cast the event_handler first into a link_Handler or we cast it upon returning it in this function


  switch_id addr() const {
    return my_addr_;
  };

public:
  flexfly_optical_switch(
    sprockit::sim_parameters* params,
    uint64_t id,
    event_manager* mgr);// : connectable_component(params,id,mgr,ty);
protected:
  void recv_payload(event* ev) {
    // cast it to a known event first?
  };

  void recv_credit(event* ev) {

  };

 private: 
  switch_id my_addr_;
  topology* top_;

  std::vector<event_handler*> outport_handler_;
  std::vector<event_handler*> inport_handler_;
  std::vector<topology::connection*> outport_connection_;
  std::vector<topology::connection*> inport_connection_;

  int num_ports_;

};


}
}

//#endif
