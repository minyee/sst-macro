/**
 * Author: Min Yee Teh
 * This is the header file containing all the function footprint for the flexfly topology class
 * which is really a canonical Dragonfly
 */
#include <sstmac/hardware/topology/flexfly_topology.h>
#include <math>
#include <algorithm>
#include <vector>


namespace sstmac{
namespace hw {

class flexfly_topology : public structured_topology {

 FactoryRegister("flexfly", topology, flexfly_topology);
protected:
  struct link {
    switch_id dest_sid; // switch_id of the destination switch
    int dest_inport; // port of the destination switch
    link_type type; 
  }
public:
 ~flexfly_topology();
   /**** BEGIN PURE VIRTUAL INTERFACE *****/
  /**
   * @brief Whether all network ports are uniform on all switches,
   *        having exactly the same latency/bandwidth parameters.
   *        If a 3D torus, e.g., has X,Y,Z directions exactly the same,
   *        this returns true.
   * @return
   */
  virtual bool uniform_network_ports() const final { //DONE
  	return false;
  };

  /**
   * @brief Whether all switches are the same, albeit with each port on the switch
   *        having slightly different latency/bandwidth configurations
   * @return
   */
  virtual bool uniform_switches_non_uniform_network_ports() const final{ //DONE
  	//no this is false because we have both electrical and optical switches
  	return false; 
  };

  /**
   * @brief Whether all switches are the same and all ports on those switches
   *        have exactly the same configuration
   * @return
   */
  virtual bool uniform_switches() const final { //DONE
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
                     std::vector<topology::connection>& conns) const; //DONE

  /**
   * @brief configure_individual_port_params.  The port-specific parameters
   *        will be stored in new namespaces "portX" where X is the port number
   * @param src
   * @param [inout] switch_params
   */
  virtual void configure_individual_port_params(switch_id src,
          sprockit::sim_parameters* switch_params) const; //DONE (RECHECK)

  /**
     For indirect networks, this includes all switches -
     those connected directly to nodes and internal
     switches that are only a part of the network
     @return The total number of switches
  */
  virtual int num_switches() const { //DONE
  	return num_groups_ * switches_per_group_ + num_optical_switches_;
  };

  /**
   * @brief max_switch_id Depending on the node indexing scheme, the maximum switch id
   *  might be larger than the actual number of switches.
   * @return The max switch id
   */
  virtual switch_id max_switch_id() const { // DONE
    return max_switch_id_;
  };

  /**
   * @brief swithc_id_slot_filled
   * @param sid
   * @return Whether a switch object should be built for a given switch_id
   */
  virtual bool switch_id_slot_filled(switch_id sid) const = 0;

  virtual int num_nodes() const = 0;

  /**
   * @brief max_node_id Depending on the node indexing scheme, the maximum node id
   *  might be larger than the actual number of nodes.
   * @return The max node id
   */
  virtual node_id max_node_id() const = 0;

  /**
   * @brief node_id_slot_filled
   * @param nid
   * @return Whether a node object should be built for a given node_id
   */
  virtual bool node_id_slot_filled(node_id nid) const = 0;

  virtual switch_id max_netlink_id() const = 0;

  virtual bool netlink_id_slot_filled(node_id nid) const = 0;

  /**
   * @brief num_endpoints To be distinguished slightly from nodes.
   * Multiple nodes can be grouped together with a netlink.  The netlink
   * is then the network endpoint that injects to the switch topology
   * @return
   */
  virtual int num_netlinks() const = 0;

  /**
   * @brief Return the maximum number of ports on any switch in the network
   * @return
   */
  virtual int max_num_ports() const {
  	return std::max(optical_switch_radix_, electrical_switch_radix_);
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
        netlink_id nodeaddr, uint16_t& switch_port) const = 0;

  /**
     For a given node, determine the ejection switch
     All messages to this node eject into the network
     through this switch
     @param nodeaddr The node to eject from
     @param switch_port [inout] The port on the switch the node ejects on
     @return The switch that ejects into the node
  */
  virtual switch_id netlink_to_ejection_switch(
        netlink_id nodeaddr, uint16_t& switch_port) const = 0;

  /**
   * @brief configure_vc_routing  Configure the number of virtual channels
   *        required for all supported routing algorithms
   * @param [inout] m
   */
  virtual void configure_vc_routing(std::map<routing::algorithm_t, int>& m) const = 0;

  /**
   * @brief node_to_ejection_switch Given a destination node,
   *        figure out which switch has an ejection connection to it
   * @param addr
   * @param port  The port number on the switch that leads to ejection
   *              to the particular node
   * @return
   */
  virtual switch_id node_to_ejection_switch(node_id addr, uint16_t& port) const = 0;

  virtual switch_id node_to_injection_switch(node_id addr, uint16_t& port) const = 0;

  /**
    This gives the minimal distance counting the number of hops between switches.
    @param src. The source switch.
    @param dest. The destination switch.
    @return The number of hops to final destination
  */
  virtual int minimal_distance(switch_id src, switch_id dst) const;
  	//return topology_diameter_;

  /**
    This gives the minimal distance counting the number of hops between switches.
    @param src. The source node.
    @param dest. The destination node.
    @return The number of hops to final destination
  */
  virtual int num_hops_to_node(node_id src, node_id dst) const = 0;

  /**
     For a given input switch, return all nodes connected to it.
     This return vector might be empty if the
     switch is an internal switch not connected to any nodes
     @return The nodes connected to switch for injection
  */
  virtual void nodes_connected_to_injection_switch(switch_id swid,
                          std::vector<injection_port>& nodes) const = 0;

  /**
     For a given input switch, return all nodes connected to it.
     This return vector might be empty if the
     switch is an internal switch not connected to any nodes
     @return The nodes connected to switch for ejection
  */
  virtual void nodes_connected_to_ejection_switch(switch_id swid,
                          std::vector<injection_port>& nodes) const = 0;

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
  virtual void minimal_route_to_switch(
    switch_id current_sw_addr,
    switch_id dest_sw_addr,
    routable::path& path) const;

  virtual bool node_to_netlink(node_id nid, node_id& net_id, int& offset) const = 0;
  
protected:
 flexfly_topology(sprockit::sim_parameters* params) : topology(params); 

 void configure_optical_or_electrical_port_params(switch_id swid, const std::string& str, sprockit::sim_parameters* sim_params) const;

private:
 // wires the dragonfly or flexfly using  
 void dfly_wire(); // NOTE: CAN BE IMPLEMENTED AT A LATER TIME

 uint32_t num_groups_; // equivalent to parameter g in Kim's paper
 uint32_t switches_per_group_; // equivalent to parameter a in Kim's paper
 uint32_t node_per_switch_; // equivalent to parameter p in Kim's paper

 uint32_t topology_diameter_; // need to figure out if the diameter needs to take into account optical hops or not
 
 //int intra_group_diameter_;
 uint32_t num_optical_switches_;

 uint32_t num_total_switches_;

 uint32_t num_optical_switches_per_group_;
 
 switch_id max_switch_id_;
//maps a switch_id to a vector of connections of said switch
 std::unordered_map<switch_id, std::vector<switch_port_pair*>> switch_connection_map_;
//maps a switch_id (must be electrical) to a vector of all the nodes (end-point compute) it is connected to
 std::unordered_map<switch_id, std::vector<injection_port*>> node_connection_map_;

 void setup_flexfly_topology();

 bool connect_switches(switch_id src, switch_id dst, link_type ltype);

 //std::vector<flexfly_optical_switch*> optical_switches_;

 //std::vector<flexfly_electrical_switch*> electrical_switches_;

public:
 int num_groups() {
 	return num_groups_;
 }
 
 int switches_per_group() {
 	return switches_per_group_;
 }

 inline bool valid_switch_id(switch_id id);
};



}
}
