/**
 * Author: Min Yee Teh
 */
#include <vector>
#include <unordered_map>
#include <sstmac/common/node_address.h>
#include <sprockit/sim_parameters.h>
#include <stdlib.h>
#include <sstmac/hardware/topology/topology.h>
#include "exacomm_dragonfly_topology.h"

namespace sstmac {
namespace hw {

 
 exacomm_dragonfly_topology::exacomm_dragonfly_topology(sprockit::sim_parameters* params) : 
                              structured_topology(params,InitMaxPortsIntra::I_Remembered, 
                                                  InitGeomEjectID::I_Remembered) {
 	num_groups_ = params->get_int_param("groups"); // controls g
 	switches_per_group_ = params->get_int_param("switches_per_group"); // controls a
  optical_links_per_switch_ = params->get_int_param("optical_links_per_switch");
 	nodes_per_switch_ = params->get_optional_int_param("nodes_per_switch", 4);
 	num_optical_switches_ = switches_per_group_;
  max_switch_id_ = num_optical_switches_ + (num_groups_ * switches_per_group_) - 1;
  outgoing_adjacency_matrix_.resize(num_groups_ * switches_per_group_);
  incoming_adjacency_matrix_.resize(num_groups_ * switches_per_group_);
 };

 exacomm_dragonfly_topology::~exacomm_dragonfly_topology() {};

 /**
  * IMPORTANT: This function will route the minimal path
  **/
 void exacomm_dragonfly_topology::minimal_route_to_switch(switch_id src_switch_addr, 
 												switch_id dst_switch_addr, 
 												routable::path& path) const {
  int src_group = src_switch_addr/switches_per_group_;
  int dst_group = src_switch_addr/switches_per_group_;
  // both the source and dest switch cannot be the same, because the caller 
  // should not call this function if this were the case
  assert(src_switch_addr != dst_switch_addr);
  // if both nodes are connected to the same switch
  if (src_group == dst_group) {
    std::vector<topology::connection> conns;  
    connected_outports(src_switch_addr, conns);
    for (int i = 0; i < conns.size(); i++) {
      if (conns[i].dst == dst_switch_addr) {
        path.set_outport(i);
        return;
      }
    }
  } else {
  }

 };

 // DONE
 void exacomm_dragonfly_topology::configure_individual_port_params(switch_id src,
          												 sprockit::sim_parameters* switch_params) const {
 	if (valid_switch_id(src)) {
 		std::string str;
 		if (is_optical_switch(src)) {
 			str = "optical";
 		} else {
 			str = "electrical";
 		}
 		configure_optical_or_electrical_port_params(src, str, switch_params);
 	} 	
 };


void exacomm_dragonfly_topology::connected_outports(const switch_id src, 
                                            std::vector<topology::connection>& conns) const {
  std::unordered_map<switch_id, std::vector<switch_link*>>::const_iterator got = switch_outport_connection_map_.find(src);
  int cidx = 0;
  if (got != switch_outport_connection_map_.end()) {
    const std::vector<switch_link*>& switch_link_vectors = got->second;
    for (switch_link* current_switch_link : switch_link_vectors) {
      conns.push_back(topology::connection());
      conns[cidx].src = src;
      conns[cidx].dst = current_switch_link->dest_sid;
      conns[cidx].src_outport = cidx; 
      conns[cidx].dst_inport = current_switch_link->dest_inport;
      conns[cidx].link_type = current_switch_link->type;
      cidx++;
    }
  }
}


bool exacomm_dragonfly_topology::switch_id_slot_filled(switch_id sid) const {
  return (sid < max_switch_id_);
}

 void exacomm_dragonfly_topology::configure_vc_routing(std::map<routing::algorithm_t, int>& m) const {
  m.insert({routing::minimal, 3});
  m.insert({routing::minimal_adaptive, 3});
  m.insert({routing::valiant, 3});
  m.insert({routing::ugal, 3});
  return;
 };

  switch_id exacomm_dragonfly_topology::node_to_ejection_switch(node_id addr, uint16_t& port) const {
    switch_id swid = addr / nodes_per_switch_; // this gives us the switch id of the switch node addr is connected to
    std::unordered_map<switch_id, std::vector<switch_link*>>::const_iterator tmp_iter = switch_outport_connection_map_.find(swid);
    const std::vector<switch_link*>& conn_vector = tmp_iter->second;
    port = std::max((int) (conn_vector.size() - 1), 0) + ((int) swid) * nodes_per_switch_; // CHECK THIS AGAIN
    return swid;
  };
  
  switch_id exacomm_dragonfly_topology::node_to_injection_switch(node_id addr, uint16_t& port) const {
    return node_to_ejection_switch(addr, port);
  };

  /**
   * NOTE: This method does not include the hop to an optical switch
   **/
  int exacomm_dragonfly_topology::minimal_distance(switch_id src, switch_id dst) const {
    //std::cout << "src switch id: " << std::to_string(src) << " dst switch id: " << std::to_string(dst) << std::endl;
    int src_group = group_from_swid(src);
    int dst_group = group_from_swid(dst);
    if (src == dst) { // same switch
      return 0;
    } else if (src_group == dst_group) { // same group
      return 1;
    } else { // different group but can reach either by 1 global and 1 local or 1 local and then 1 global
      std::unordered_map<switch_id, std::vector<switch_link*>>::const_iterator tmp_iter = switch_outport_connection_map_.find(src);
      const std::vector<switch_link*>& conn_vector = tmp_iter->second;
      bool two_or_three = true; 
      
      for (switch_link* tmp_link : conn_vector ) {
        if (tmp_link->type == electrical)
          continue;
      }
      return two_or_three ? 2 : 3;
    }
  };

  int exacomm_dragonfly_topology::num_hops_to_node(node_id src, node_id dst) const {
    int src_swid = src / (nodes_per_switch_);
    int dst_swid = dst / (nodes_per_switch_);
    int min_dist = distance_matrix_[src_swid][dst_swid];
    return min_dist + 2; // added by 2 because each node is 1 hop away from it's switch
  };

  void exacomm_dragonfly_topology::nodes_connected_to_injection_switch(switch_id swid, 
                                                              std::vector<injection_port>& nodes) const {
    int i = 0;
    int port_offset = switches_per_group_;
    for (int i = 0; i < nodes_per_switch_; i++) {
      int node_id = swid * nodes_per_switch_ + i;
      int port_ind = port_offset + i;
      injection_port ijp;
      ijp.nid = node_id;
      ijp.port = port_ind;
      nodes.push_back(ijp);
    }

  };

  void exacomm_dragonfly_topology::nodes_connected_to_ejection_switch(switch_id swid, 
                                                              std::vector<injection_port>& nodes) const { 
    exacomm_dragonfly_topology::nodes_connected_to_injection_switch(swid, nodes);
  };

  // returns the group id of a given switch
  int exacomm_dragonfly_topology::group_from_swid(switch_id swid) const {
    return swid / (switches_per_group_);
  };

  /**
   * prints out the all the connections for each switch
   */
  void exacomm_dragonfly_topology::print_port_connection_for_switch(switch_id swid) const {
    std::unordered_map<switch_id, std::vector<switch_link*>>::const_iterator tmp_iter = 
                                        switch_outport_connection_map_.find(swid);
    if (tmp_iter == switch_outport_connection_map_.end()) {
      return;
    }

    const std::vector<switch_link*>& connection_vector = tmp_iter->second;

    std::stringstream message;
    int i = 0;
    for (switch_link* sl_ptr : connection_vector) {
      // check if null, if null have to throw an error 
      if (sl_ptr) {
        message << "Dest switch_id: " << std::to_string(sl_ptr->dest_sid);
        message << " Dest inport: " << std::to_string(sl_ptr->dest_inport);
        message << " Link type: ";
        if (sl_ptr->type == Link_Type::electrical) {
          message << "ELECTRICAL" << std::endl;
        } else {
          message << "OPTICAL" << std::endl;
        }
      } else {
        spkt_abort_printf("A switch link with swid: %d is null\n", swid);
      }
      i++;
    }
    std::cout << message.str(); 
  };


  /**
   * @brief num_endpoints To be distinguished slightly from nodes.
   * Multiple nodes can be grouped together with a netlink.  The netlink
   * is then the network endpoint that injects to the switch topology
   * @return
   */
  int exacomm_dragonfly_topology::num_netlinks() const {
    return 1;
  }; 

  switch_id exacomm_dragonfly_topology::max_netlink_id() const {
    return max_switch_id_;
  };

  bool exacomm_dragonfly_topology::netlink_id_slot_filled(node_id nid) const {
    return true;
  };

  /**
     For a given node, determine the injection switch
     All messages from this node inject into the network
     through this switch
     @param nodeaddr The node to inject to
     @param switch_port [inout] The port on the switch the node injects on
     @return The switch that injects from the node
  */
  switch_id exacomm_dragonfly_topology::netlink_to_injection_switch(
        netlink_id nodeaddr, uint16_t& switch_port) const {
    return max_switch_id_;
  };

  /**
     For a given node, determine the ejection switch
     All messages to this node eject into the network
     through this switch
     @param nodeaddr The node to eject from
     @param switch_port [inout] The port on the switch the node ejects on
     @return The switch that ejects` into the node
  */
  switch_id exacomm_dragonfly_topology::netlink_to_ejection_switch(
        netlink_id nodeaddr, uint16_t& switch_port) const {
    return max_switch_id_;
  };

  
  bool exacomm_dragonfly_topology::node_to_netlink(node_id nid, node_id& net_id, int& offset) const {
    return true;
  };

  switch_id exacomm_dragonfly_topology::node_to_switch(node_id nid) const {
    switch_id swid = nid / (nodes_per_switch_);
    return swid;
  };

  /**
   * This is the key function that will be called by the electrical switches
   **/
  void exacomm_dragonfly_topology::route_minimal(int src_switch, int dst_switch, flexfly_packet* f_packet) {
    assert(src_switch != dst_switch);
    if (!updated_routing_table_) {
      // if haven't updated yet, just use the routing table
      flexfly_path *flexpath = routing_table_[src_switch][dst_switch];
      f_packet->set_path(flexpath);
      return;
    } else {
      // reroute the network here
    }
  };

  void exacomm_dragonfly_topology::set_connection(int src_switch, int port_num, int dst_swtich, bool inport_or_outport) {
    if (inport_or_outport) {
      if (incoming_adjacency_matrix_[src_switch] == 0) 
        incoming_adjacency_matrix_[src_switch].resize(num_groups_ * switches_per_group_);
      incoming_adjacency_matrix_[src_switch][port_num] = dst_switch;
    } else {
      if (outgoing_adjacency_matrix_[src_switch] == 0) 
        outgoing_adjacency_matrix_[src_switch].resize(num_groups_ * switches_per_group_);
      outgoing_adjacency_matrix_[src_switch][port_num] = dst_switch;
    }
    return;
  }
}
}

