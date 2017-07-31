/**
 * Author: Min Yee Teh
 *
 */

//#ifndef SSTMAC_HARDWARE_NETWORK_SWITCHES_NETWORKSWITCH_H_INCLUDED
//#define SSTMAC_HARDWARE_NETWORK_SWITCHES_NETWORKSWITCH_H_INCLUDED



#include <sprockit/factories/factory.h>
#include <sprockit/debug.h>

#include <sstmac/hardware/common/connection.h> // have to import connection because it contains the prototype for connectable_component
                                              // which flexfly_optical_switch inherits
#include <sstmac/hardware/router/router_fwd.h>
#include <sstmac/hardware/topology/topology_fwd.h>
#include <sstmac/hardware/topology/topology.h>
#include <sstmac/common/event_scheduler.h>
#include <sprockit/sim_parameters_fwd.h>

#if SSTMAC_INTEGRATED_SST_CORE
#include <sstmac/sst_core/integrated_component.h>
#endif

#include <vector>

DeclareDebugSlot(network_switch)
#define switch_debug(...) \
  debug_printf(sprockit::dbg::network_switch, "Switch %d: %s", \
    int(addr()), sprockit::printf(__VA_ARGS__).c_str())


template <class T, class Fxn>
sstmac::link_handler*
new_link_handler(const T* t, Fxn fxn){
  return new SST::Event::Handler<T>(const_cast<T*>(t), fxn);
}

//template<class T, class Fxn> sstmac::link_handler* new_link_handler(const T* t, Fxn function) {
  //return new SST::Event::Handler<T> const_cast(function);
//}

namespace sstmac {
namespace hw {
//template<link_handler*> new_link_handler// need a functor that returns a link_handler from just an event_handler
/**
 * @brief The network_switch class
 * A class encapsulating a network switch that packets must traverse on the network.
 * The network switch performs both routing computations and congestion modeling.
 */
class flexfly_optical_switch :
  public connectable_component
{
  DeclareFactory(network_switch,uint64_t,event_manager*)
 public:
  virtual ~flexfly_optical_switch();

  virtual std::string to_string() const {
    return "Flexfly Optical Switch";
  };

  virtual void init(unsigned int phase) override;

  virtual void deadlock_check() override;

  virtual void deadlock_check(event* ev) override;

  virtual void setup(); //needed for SST core compatibility
  
  // It is very crucial to know that the link_handlers have to be both on the receiving end.
  // That means that the methods/member functions that return link_handlers pointers have to 
  // be on the receiving end

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
                                event_handler* payload_handler) {
    outport_link_handler_[dst_inport] = new_link_handler(this, payload_handler);
  };


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
                              event_handler* credit_handler) {
    // tying an event handler to an input port?
    inport_link_handler_[src_outport] = new_link_handler(this, credit_handler);

  };

  /**
   * @brief credit_handler
   * @param port
   * @return Can be null, if no credits are ever to be received
   */
  virtual link_handler* credit_handler(int port) const {
    // this is the error-catching part
    if (port < 0 || port >= num_ports_) {
      return nullptr;
    }
    return inport_link_handler_[port];
  };

  /**
   * @brief payload_handler
   * @param port
   * @return
   */
  virtual link_handler* payload_handler(int port) const {
    // this is the error-catching part
    if (port < 0 || port >= num_ports_) {
      return nullptr;
    }
    return outport_link_handler_[port]; // either if we cast the event_handler first into a link_Handler or we cast it upon returning it in this function
  };

  switch_id addr() const {
    return my_addr_;
  };


 protected:
  flexfly_optical_switch(
    sprockit::sim_parameters* params,
    uint64_t id,
    event_manager* mgr,
    device_id::type_t ty = device_id::router);// : connectable_component(params,id,mgr,ty);

  void recv_payload(event* ev) {
    // cast it to a known event first?
  };

 private: 
  switch_id my_addr_;
  topology* top_;

  std::vector<link_handler*> outport_link_handler_;
  std::vector<link_handler*> inport_link_handler_;
  std::vector<topology::connection*> outport_connection_;
  std::vector<topology::connection*> inport_connection_;

  int num_ports_;

};


}
}

//#endif
