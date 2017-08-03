#include <sstmac/hardware/common/connection.h>
#include <sst/core/model/element_python.h>
#include <sst/core/element.h>
#include <iostream>
#include <sprockit/malloc.h>

using namespace sstmac;
using namespace sstmac::hw;
using namespace SST;


/**
For creating custom hardware components inside of SST/macro, it follows
essentially the same steps as a regular SST component. However, SST/macro
provides an extra wrapper layer and present a slightly different interface
for connecting objects together in the simulated network. This 'connectable'
interface is presented below.
*/

/**
No special Python actions are needed so this is null.
*/
char py[] = {0x00};

typedef uint64_t credit_t;

/**
 * @brief The TestModule class
 * SST will look for this module information after loading libtest.so using dlopen
 * dlopen of libtest.so is triggered by running 'import sst.test'
 * inside the Python configuration script
 */
class TestModule : public SSTElementPythonModule {
 public:
  TestModule(std::string library) :
    SSTElementPythonModule(library)
  {
    addPrimaryModule(py);
  }

  SST_ELI_REGISTER_PYTHON_MODULE(
   TestModule,
   "test",
   SST_ELI_ELEMENT_VERSION(1,0,0)
  )

};

/*
template <class T, class Fxn>
link_handler*
new_link_handler(const T* t, Fxn fxn){
  return new SST::Event::Handler<T>(const_cast<T*>(t), fxn);
}
*/

/**
 * @brief The test_component class
 * For sst-macro, all the componennts must correspond to a parent factory type
 * in this case, we create a factory type test_component from which all
 * test components will inherit
 */
class test_component : public connectable_component {
 public:
  DeclareFactory(test_component,uint64_t,event_manager*)

  /**
   * @brief test_component Standard constructor for all components
   *  with 3 basic parameters
   * @param params  All of the parameters scoped for this component
   * @param id      A unique ID for this component
   * @param mgr     The event manager that will schedule events for this component
   */
  test_component(sprockit::sim_parameters* params, uint64_t id, event_manager* mgr) :
    connectable_component(params,id,
      device_id(params->get_int_param("id"),device_id::router),mgr)
 {
 }
};


/** 
 *  A new type of event that contains different types of packet messages
 *
 */
class jason_test_event : public event {
  ImplementSerializable(jason_test_event);

 public:
  enum Packet_Type {CREDIT, PAYLOAD};
  /**
   * the default constructor
   */
  jason_test_event() {packet_type_ = PAYLOAD;};
  jason_test_event(Packet_Type pack_type) {
    packet_type_ = pack_type;
    if (pack_type == CREDIT) {
      credit_ = ~(0);
    } else {
      credit_ = 0;
    }
  };
  ~jason_test_event() {};

  Packet_Type packet_type() const {
    return packet_type_;
  };

  credit_t get_credit() const {
    return credit_;
  }

  private:
  Packet_Type packet_type_;
  credit_t credit_;

};




class test_event : public event {
 public:
  //Make sure to satisfy the serializable interface
  ImplementSerializable(test_event) //WHAT IS THIS EXACTLY?
};

/**
 * @brief The dummy_switch class
 * This is a basic instance of a test component that will generate events
 */
class dummy_switch : public test_component {
 public:
  
  RegisterComponent("dummy", test_component, dummy_switch,
           "test", COMPONENT_CATEGORY_NETWORK,
           "A dummy switch for teaching")

  /**
   * @brief dummy_switch Standard constructor for all components
   *  with 3 basic parameters
   * @param params  All of the parameters scoped for this component
   * @param id      A unique ID for this component
   * @param mgr     The event manager that will schedule events for this component
   */
  dummy_switch(sprockit::sim_parameters* params, uint64_t id, event_manager* mgr) :
   test_component(params,id,mgr), id_(id)
  {
    //make sure this function gets called
    //unfortunately, due to virtual function initialization order
    //this has to be called in the base child class
    init_links(params);
    //init params
    port_num_ = params->get_optional_int_param("port_num", 1);
    num_ping_pongs_ = params->get_optional_int_param("num_ping_pongs", 2);
    //std::cout << "THE INITIALIZATION PROCESS GIVES num_ping_pongs_ as " + std::to_string(num_ping_pongs_) << std::endl;
    latency_ = params->get_time_param("latency");
    credits_ = (credit_t *) malloc(port_num_ * sizeof(credit_t));
    for (int i = 0; i < port_num_; ++i) {
      credits_[i] = ~(0); // an equivalent on MAX_UNSIGNED_INT that is 64 bits
    }
  }

  std::string to_string() const override { return "dummy";}

  void recv_payload(event* ev){
    
    std::cout << "Owh hey, component: " + std::to_string(id_) +
                   " has received a packet" << std::endl;
    jason_test_event* tev = dynamic_cast<jason_test_event*>(ev); // casting an event specifically into a test event
    if (tev == nullptr){
      std::cerr << "received wrong event type" << std::endl;
      abort();
    }
    //std::cout << "The class_id is: " + std::to_string(tev->cls_id()) << std::endl;
    //send_credit();
    if (num_ping_pongs_ > 0){
      send_ping_message();
    }
  }

  void recv_credit(event* ev){
    jason_test_event* tev = dynamic_cast<jason_test_event*>(ev);

    if (!tev || tev->packet_type() != jason_test_event::CREDIT) {
      std::cerr << "SHIT SHIT SHIT" << std::endl;
      abort();
    }
    std::cout << "CREDIT at component_id: " + std::to_string(id_) + " has been received" << std::endl; 
    // at this point, credit has been successfully verified
  }

  void connect_output(
    sprockit::sim_parameters* params,
    int src_outport,
    int dst_inport,
    event_handler* handler) override {
    //register handler on port
    partner_ = handler;
    std::cout << "Connecting output "
              << src_outport << "-> " << dst_inport
              << " on component " << id_ << std::endl;
  }

  void connect_input(
    sprockit::sim_parameters* params,
    int src_outport,
    int dst_inport,
    event_handler* handler) override {
    //we won't do anything with credits, but print for tutorial
    partner_input_ = handler; // NEWLY ADDED
    std::cout << "Connecting input "
              << src_outport << "-> " << dst_inport
              << " on component " << id_ << std::endl;
  }

  void setup() override {
    std::cout << "Setting up " << id_ << std::endl;
    //make sure to call parent setup method
    test_component::setup();
    //send an initial test message
    send_credit();

    send_ping_message();
  }

  void init(unsigned int phase) override {
    std::cout << "Initializing " << id_
              << " on phase " << phase << std::endl;
    //make sure to call parent init method
    test_component::init(phase);
  }

  link_handler* credit_handler(int port) const override {
    std::cout << "credit_handler for component_id: " + std::to_string(id_) + " is being called" << std::endl;
    //return nullptr;
    return new_link_handler(this, &dummy_switch::recv_credit);
  }

  link_handler* payload_handler(int port) const override {
    std::cout << "payload_handler for component_id: " + std::to_string(id_) + " is being called" << std::endl; 
    //return nullptr;
    return new_link_handler(this, &dummy_switch::recv_payload);
  }

 private:
  void send_ping_message(){
      send_to_link(partner_, new jason_test_event(jason_test_event::PAYLOAD));
      --num_ping_pongs_;
  }

  void send_credit() {
    std::cout << "SENDING ACKLNOWLEDGE PACKET TO THE SENDER" << std::endl;
    send_to_link(partner_input_, new jason_test_event(jason_test_event::CREDIT));
    return;
  }


 private:
  int port_num_;
  credit_t* credits_; // associates the input port of a switch with a given 
                      // amont of credit
  event_handler* partner_input_; //NEWLY ADDED
  event_handler* partner_;
  timestamp latency_;
  int num_ping_pongs_;
  uint64_t id_;

  //int queue_length_;
};

