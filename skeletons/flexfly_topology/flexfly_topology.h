/**
 * Author: Min Yee Teh
 * This is the header file containing all the function footprint for the flexfly topology class
 * which is really a canonical Dragonfly
 */
//#include <sstmac/hardware/topology/flexfly_topology.h>
//#include <math>
#include <algorithm>
#include <vector>
#include <sstmac/hardware/topology/topology.h>
#include <sstmac/hardware/topology/structured_topology.h>
#include <unordered_map>

namespace sstmac{
namespace hw {

class flexfly_topology : public structured_topology {
public:
FactoryRegister("flexfly", topology, flexfly_topology, "This is flexfly topology for Flexfly project");
/*
RegisterComponent("flexfly", topology, flexfly_topology,
           "topol", COMPONENT_CATEGORY_NETWORK,
           "The flexfly topology")
*/
// this is for the structured_topology inheritance since structured_topology requires diameter
virtual int diameter() const override {return 3;};

protected:
  struct switch_link {
    switch_id dest_sid; // switch_id of the destination switch
    int dest_inport; // port of the destination switch
    Link_Type type; 
  };

public:
 virtual std::string to_string() const override {
  return "flexfly_topology";
 };

 ~flexfly_topology();
   /**** BEGIN PURE VIRTUAL INTERFACE *****/
  /**
   * @brief Whether all network ports are uniform on all switches,
   *        having exactly the same latency/bandwidth parameters.
   *        If a 3D torus, e.g., has X,Y,Z directions exactly the same,
   *        this returns true.
   * @return
   */
  virtual bool uniform_network_ports() const override { //DONE
  	return false;
  };

  /**
   * @brief Whether all switches are the same, albeit with each port on the switch
   *        having slightly different latency/bandwidth configurations
   * @return
   */
  virtual bool uniform_switches_non_uniform_network_ports() const override{ //DONE
  	//no this is false because we have both electrical and optical switches
  	return false; 
  };

  /**
   * @brief Whether all switches are the same and all ports on those switches
   *        have exactly the same configuration
   * @return
   */
  virtual bool uniform_switches() const override { //DONE
  	return false;
  };

  /**
   * @brief connected_outports
   *        Given a 3D torus e.g., the connection vector would contain
   *        6 entries, a +/-1 for each of 3 dimensions.
   * @param src   Get the source switch in the connection
   * @param conns The set of output connections with dst switch_id
   *              and the port numbers for each connection
   */
  virtual void connected_outports(switch_id src,
                     std::vector<topology::connection>& conns) const override; //DONE

  /**
   * @brief configure_individual_port_params.  The port-specific parameters
   *        will be stored in new namespaces "portX" where X is the port number
   * @param src
   * @param [inout] switch_params
   */
  virtual void configure_individual_port_params(switch_id src,
          sprockit::sim_parameters* switch_params) const override; //DONE (RECHECK)

  /**
     For indirect networks, this includes all switches -
     those connected directly to nodes and internal
     switches that are only a part of the network
     @return The total number of switches
  */
  virtual int num_switches() const override { //DONE
    std::cout << "num_switches?" << std::endl;
  	return num_groups_ * switches_per_group_ + num_optical_switches_;
  };

  /**
   * @brief max_switch_id Depending on the node indexing scheme, the maximum switch id
   *  might be larger than the actual number of switches.
   * @return The max switch id
   */
  virtual switch_id max_switch_id() const override{ // DONE
    std::cout << "max_switch_id" << std::endl;
    return max_switch_id_;
  };

  /**
   * @brief swithc_id_slot_filled
   * @param sid
   * @return Whether a switch object should be built for a given switch_id
   */
  virtual bool switch_id_slot_filled(switch_id sid) const override; // DONE

  virtual int num_nodes() const override { // DONE
    int node_num = num_groups_ * switches_per_group_ * nodes_per_switch_;
    //std::cout << "num_nodes?" << std::endl;
    return node_num;
  };

  /**
   * @brief max_node_id Depending on the node indexing scheme, the maximum node id
   *  might be larger than the actual number of nodes.
   * @return The max node id
   */
  virtual node_id max_node_id() const override{ // DONE
    std::cout << "max_node_id?" << std::endl;
    return max_node_id_;
  };

  /**
   * @brief node_id_slot_filled
   * @param nid
   * @return Whether a node object should be built for a given node_id
   */
  virtual bool node_id_slot_filled(node_id nid) const override { // DONE
    std::cout << "node_id_slot_filled?" << std::endl;
    return (nid < flexfly_topology::max_node_id());
  };

  virtual switch_id max_netlink_id() const override; // DONE (RECHECK)

  virtual bool netlink_id_slot_filled(node_id nid) const override; // DONE (RECHECK)

  /**
   * @brief num_endpoints To be distinguished slightly from nodes.
   * Multiple nodes can be grouped together with a netlink.  The netlink
   * is then the network endpoint that injects to the switch topology
   * @return
   */
  virtual int num_netlinks() const override; // DONE (RECHECK)

  /**
   * @brief Return the maximum number of ports on any switch in the network
   * @return
   */
  virtual int max_num_ports() const override { // DONE (RECHECK)
  	//return std::max(optical_switch_radix_, electrical_switch_radix_);
    return switches_per_group_ + nodes_per_switch_; // + 1 - 1
  }

  /**
     For a given node, determine the injection switch
     All messages from this node inject into the network
     through this switch
     @param nodeaddr The node to inject to
     @param switch_port [inout] The port on the switch the node injects on
     @return The switch that injects from the node
  */
  virtual switch_id netlink_to_injection_switch(
        netlink_id nodeaddr, uint16_t& switch_port) const override;

  /**
     For a given node, determine the ejection switch
     All messages to this node eject into the network
     through this switch
     @param nodeaddr The node to eject from
     @param switch_port [inout] The port on the switch the node ejects on
     @return The switch that ejects into the node
  */
  virtual switch_id netlink_to_ejection_switch(
        netlink_id nodeaddr, uint16_t& switch_port) const override;

  /**
   * @brief configure_vc_routing  Configure the number of virtual channels
   *        required for all supported routing algorithms
   * @param [inout] m
   */
  virtual void configure_vc_routing(std::map<routing::algorithm_t, int>& m) const override; // DONE (RECHECK)

  /**
   * @brief node_to_ejection_switch Given a destination node,
   *        figure out which switch has an ejection connection to it
   * @param addr
   * @param port  The port number on the switch that leads to ejection
   *              to the particular node
   * @return
   */
  virtual switch_id node_to_ejection_switch(node_id addr, uint16_t& port) const override; // DONE (RECHECK)

  virtual switch_id node_to_injection_switch(node_id addr, uint16_t& port) const override; // DONE (RECHECK)

  /**
    This gives the minimal distance counting the number of hops between switches.
    @param src. The source switch.
    @param dest. The destination switch.
    @return The number of hops to fibal destination
  */
  virtual int minimal_distance(switch_id src, switch_id dst) const override; // DONE (PLEASE RECHECK)
  	//return topology_diameter_;

  /**
    This gives the minimal distance counting the number of hops between switches.
    @param src. The source node.
    @param dest. The destination node.
    @return The number of hops to final destination
  */
  virtual int num_hops_to_node(node_id src, node_id dst) const override;

  /**
     For a given input switch, return all nodes connected to it.
     This return vector might be empty if the
     switch is an internal switch not connected to any nodes
     @return The nodes connected to switch for injection
  */
  virtual void nodes_connected_to_injection_switch(switch_id swid,
                          std::vector<injection_port>& nodes) const override;

  /**
     For a given input switch, return all nodes connected to it.
     This return vector might be empty if the
     switch is an internal switch not connected to any nodes
     @return The nodes connected to switch for ejection
  */
  virtual void nodes_connected_to_ejection_switch(switch_id swid,
                          std::vector<injection_port>& nodes) const override;

  virtual int num_leaf_switches() const override {
    return num_groups_ * switches_per_group_;
  };
  /**
     Given the current location and a destination,
     compute the minimal path to the destination.
     The path generally consists of a dimension,
     a direction or branch along that dimension,
     a port number for that dim/dir combination,
     and a virtual channel along the dim/dir
     appropriate for avoiding deadlock.
     @param current_sw_addr The addr of the current switch
     @param dest_sw_addr The addr of the destination switch
     @param path [inout] A complete path descriptor to the destination switch
  */
  virtual void minimal_route_to_switch( //this is really the key
    switch_id current_sw_addr,
    switch_id dest_sw_addr,
    routable::path& path) const override;

  virtual bool node_to_netlink(node_id nid, node_id& net_id, int& offset) const override;
  
  virtual void configure_metis(metis_config* configuration) const override;

  void print_topology() const;

  void print_connectivity_matrix() const; 

  flexfly_topology(sprockit::sim_parameters* params); 

  switch_id node_to_logp_switch(node_id nid) const;
protected:
 
 void configure_optical_or_electrical_port_params(switch_id swid, std::string& str, sprockit::sim_parameters* sim_params) const;

private:
 // wires the dragonfly or flexfly using  
 void dfly_wire(std::string& filename); // NOTE: CAN BE IMPLEMENTED AT A LATER TIME

 uint32_t num_groups_; // equivalent to parameter g in Kim's paper
 uint32_t switches_per_group_; // equivalent to parameter a in Kim's paper
 uint32_t nodes_per_switch_; // equivalent to parameter p in Kim's paper

 //uint32_t topology_diameter_; // need to figure out if the diameter needs to take into account optical hops or not
 
 //int intra_group_diameter_;
 uint32_t num_optical_switches_;

 uint32_t num_total_switches_;
 uint32_t optical_switch_radix_;
 uint32_t num_optical_switches_per_group_;
 
 switch_id max_switch_id_;

 node_id max_node_id_;
//maps a switch_id to a vector of connections of said switch
 std::unordered_map<switch_id, std::vector<switch_link*>> switch_connection_map_;
//maps a switch_id (must be electrical) to a vector of all the nodes (end-point compute) it is connected to
 //std::unordered_map<switch_id, std::vector<node_id>> node_connection_map_;

 void setup_flexfly_topology();

 void connect_switches(switch_id src, switch_id dst, Link_Type ltype);

 bool valid_switch_id(switch_id id) const {
    return id < (switches_per_group_ * num_groups_ + num_optical_switches_);
 };

 bool is_optical_switch(switch_id sid) const;

 bool is_electrical_switch(switch_id sid) const;
 //std::vector<flexfly_optical_switch*> optical_switches_;

 //std::vector<flexfly_electrical_switch*> electrical_switches_;
 
 

 // figures out if two groups are currently connected to one another
 // also accounts for the connectivity within the optical switches
 bool is_group_connected(int src_group, int dst_group) const;
 
 inline int group_from_swid (switch_id swid) const;
 
 inline switch_id public_swid_to_private_swid(switch_id swid) const; 
 
 void print_port_connection_for_switch(switch_id swid) const;


public:
 int num_groups() {
 	return num_groups_;
 }
 
 int switches_per_group() {
 	return switches_per_group_;
 }


};



}
}
