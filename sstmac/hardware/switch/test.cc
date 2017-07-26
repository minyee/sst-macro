#include <sstmac/hardware/common/connection.h>
#include <sst/core/model/element_python.h>
#include <sst/core/element.h>
#include <sprockit/sprockit/malloc.h>

using namespace sstmac;
using namespace sstmac::hw;
using namespace SST;


//sst will look for this when loading libtest.so after dlopen
class TestModule : public SSTElementPythonModule {
 public:
  TestModule(std::string library) :
    SSTElementPythonModule(library)
  {
  }

  SST_ELI_REGISTER_PYTHON_MODULE(
  TestModule,
  "test",
  SST_ELI_ELEMENT_VERSION(1,0,0)
  )

};

//for sst-macro, all the componennts you build should have a factory type
class optical_component : public connectable_component {
 public:
  DeclareFactory(optical_component,uint64_t,event_manager*)

  optical_component(sprockit::sim_parameters* params, uint64_t id, event_manager* mgr) :
    connectable_component(params,id,
      device_id(params->get_int_param("id"),device_id::router),mgr)
 {
 }
    
};

//this is the actual component that you care about
class dummy_switch : public optical_component {
 public:
  RegisterComponent("dummy", optical_component, dummy_switch,
           "test", COMPONENT_CATEGORY_NETWORK,
           "A dummy switch for teaching")

  dummy_switch(sprockit::sim_parameters* params, uint64_t id, event_manager* mgr) :
   optical_component(params,id,mgr)
  {
    output_handler_ = (link_handler*) malloc(2 * sizeof(link_handler*));
    output_handler_[0] = nullptr; // just for safety, initialize to null
    output_handler_[1] = nullptr;
  }

  virtual std::string to_string() const { return "dummy";}

  virtual void connect_output(
    sprockit::sim_parameters* params,
    int src_outport, // why source outport? maybe cause from the perspective of the switch, it's input is the source node's output?
    int dst_inport, // why dest inport?
    event_handler* handler) {
    output_handler_[dst_inport] = handler; // attach this specific handler to handle dst_inport's 
  }

  virtual void connect_input( // perhaps it's connect input port of a switch to a specific type of handler?
    sprockit::sim_parameters* params,
    int src_outport,
    int dst_inport,
    event_handler* handler) {}

  virtual link_handler* credit_handler(int port) const { 
    return nullptr;
  }

  virtual link_handler* payload_handler(int port) const {
    return output_handler_[port];
    //return nullptr;
  }

  void test_handler(event* ev) {
    
  }
protected:
  link_handler** output_handler_[];
};


/*
#include <sstmac/hardware/common/connection.h>
#include <sst/core/model/element_python.h>
#include <sst/core/element.h>

using namespace sstmac;
using namespace sstmac::hw;
using namespace SST;


//sst will look for this when loading libtest.so after dlopen
class TestModule : public SSTElementPythonModule {
 public:
  TestModule(std::string library) :
    SSTElementPythonModule(library)
  {
  }

  SST_ELI_REGISTER_PYTHON_MODULE(
  TestModule,
  "test",
  SST_ELI_ELEMENT_VERSION(1,0,0)
  )

};

//for sst-macro, all the componennts you build should have a factory type
class optical_component : public connectable_component {
 public:
  DeclareFactory(optical_component,uint64_t,event_manager*)

  optical_component(sprockit::sim_parameters* params, uint64_t id, event_manager* mgr) :
    connectable_component(params,id,
      device_id(params->get_int_param("id"),device_id::router),mgr)
 {
 }
    
};

//this is the actual component that you care about
class dummy_switch : public optical_component {
 public:
  RegisterComponent("dummy", optical_component, dummy_switch,
           "test", COMPONENT_CATEGORY_NETWORK,
           "A dummy switch for teaching")

  dummy_switch(sprockit::sim_parameters* params, uint64_t id, event_manager* mgr) :
   optical_component(params,id,mgr)
  {
  }

  virtual std::string to_string() const { return "dummy";}

  virtual void connect_output(
    sprockit::sim_parameters* params,
    int src_outport,
    int dst_inport,
    event_handler* handler) {}

  virtual void connect_input(
  sprockit::sim_parameters* params,
  int src_outport,
  int dst_inport,
  event_handler* handler) {}

  virtual link_handler* credit_handler(int port) const { 
    return nullptr;
  }

  virtual link_handler* payload_handler(int port) const {
    return nullptr;
  }

};

*/