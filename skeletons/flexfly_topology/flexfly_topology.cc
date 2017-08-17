/**
 * Author: Min Yee Teh
 */
#include <iostream>	
#include <algorithm>
#include <vector>
#include <unordered_map>
#include <sstmac/common/node_address.h>
#include <sprockit/sim_parameters.h>
#include <stdlib.h>
#include <sstmac/hardware/topology/topology.h>
#include "flexfly_topology.h"



namespace sstmac {
namespace hw {

 flexfly_topology::flexfly_topology(sprockit::sim_parameters* params) : structured_topology(params,InitMaxPortsIntra::I_Remembered, InitGeomEjectID::I_Remembered) {

 	// initialization of the private class member variables 
 	// intra_group_diameter_ = 1; // this assumes that intra group topology is all to all
 	num_groups_ = params->get_optional_int_param("groups", 7);
 	num_optical_switches_per_group_ = 1; 
 	switches_per_group_ = params->get_optional_int_param("switches_per_group", 6);
 	nodes_per_switch_ = params->get_optional_int_param("nodes_per_switch", 7);
  optical_switch_radix_ = params->get_optional_int_param("optical_switch_radix", switches_per_group_);
 	num_optical_switches_ = num_groups_ * num_optical_switches_per_group_; // for now, assume that each group will have one optical switch attached to itself.
 	num_total_switches_ = num_optical_switches_ + num_groups_ * switches_per_group_;
 	//switch_id sid = 0;
  bool force_set_opt_switch_params = 
              params->get_optional_bool_param("force_set_opt_switch_params", true);
  num_optical_switches_per_group_ = (switches_per_group_ / optical_switch_radix_) + 
                                        ((switches_per_group_ % optical_switch_radix_ == 0) ? 0 : 1);
  num_optical_switches_per_group_ = force_set_opt_switch_params ? 1 : num_optical_switches_per_group_;
  optical_switch_radix_ = force_set_opt_switch_params ? switches_per_group_ : optical_switch_radix_;
 	setup_flexfly_topology();
 }

 // need to deallocate everything at the deconstructor
 flexfly_topology::~flexfly_topology() {
  std::cout << "Flexfly_topology destructor is being called" << std::endl;
  for (const std::pair<switch_id, std::vector<switch_link*>> elem : switch_connection_map_) {
    const std::vector<switch_link*>& conn_vector = elem.second;  
    //for (auto it = switch_connection_map_.begin(); it != switch_connection_map_.end(); ++it){  
    for (auto const&  switch_link_ptr : conn_vector) {
      free(switch_link_ptr);
    }
  }

 }


 void flexfly_topology::setup_flexfly_topology() {
  // setup the intra-group all-to-all connection first.
  for (int group = 0; group < num_groups_; group++) {
    for (int intra_group_index = 0; intra_group_index < switches_per_group_ - 1; intra_group_index++) {
      switch_id swid = group * (switches_per_group_ + num_optical_switches_per_group_) + intra_group_index;
      for (int other_intra_group_indices = intra_group_index + 1; 
            other_intra_group_indices < switches_per_group_; 
            other_intra_group_indices++) {
        switch_id dest_id = group * (switches_per_group_ + num_optical_switches_per_group_) + other_intra_group_indices;
        // form all intra-group electrical connections first
        connect_switches(swid, dest_id, Link_Type::electrical); 
        connect_switches(dest_id, swid, Link_Type::electrical);
      }
    }
  }

  // tells us what the last switch was used in this group, each index represents a group
  switch_id *last_used_id = (switch_id *) std::calloc(num_groups_, sizeof(switch_id));
  // SETUP OPTICAL SWITCHES
  for (int g = 0; g < num_groups_; g++) {
    for (int opt_switch = 0; opt_switch < num_optical_switches_per_group_; opt_switch++) {
      switch_id opt_swid = (g * (switches_per_group_ + num_optical_switches_per_group_)) + switches_per_group_ + opt_switch;
      //for (int opt_radix = 0; opt_radix < optical_switch_radix_; opt_radix++) {
        int opt_radix = 0;
        for (int gg = 0; gg < num_groups_ ; gg++) {
          if (opt_radix >= optical_switch_radix_) {
            break;
          }
          if (gg == g)
            continue;
          switch_id elec_swid = gg * (switches_per_group_ + num_optical_switches_per_group_) + (int)(last_used_id[gg]);
          last_used_id[gg] = (last_used_id[gg] + 1) % switches_per_group_;
          connect_switches(opt_swid, elec_swid, Link_Type::optical);
          connect_switches(elec_swid, opt_swid, Link_Type::optical);
          opt_radix++;
        }
    }
  }
  free(last_used_id);
  max_switch_id_ = num_groups_ * (switches_per_group_ + num_optical_switches_per_group_) - 1;
  max_node_id_ = num_groups_ * switches_per_group_ * nodes_per_switch_ - 1;
 }

 void flexfly_topology::configure_metis(metis_config* configuration) const {
	if (!configuration) {
		return;
	}
	configuration->nvtxs = (idx_t) num_groups_ * switches_per_group_;
 }

 // DONE
 void flexfly_topology::connected_outports(const switch_id src, 
 											                      std::vector<topology::connection>& conns) const {
 	if (!valid_switch_id(src) || switch_connection_map_.count(src) == 0) 
 		return;
  int cidx = 0;
  std::cout << "kaninia chao jibai" << std::endl;
  std::unordered_map<switch_id, std::vector<switch_link*>>::const_iterator got = switch_connection_map_.find (src);
  //std::vector<switch_link*>& switch_link_vectors = (switch_connection_map_[src]).second;
  std::vector<switch_link*> switch_link_vectors = got->second;//->second();
  for (switch_link* current_switch_link : switch_link_vectors) {
    conns[cidx].src = src;
    conns[cidx].dst = current_switch_link->dest_sid;
    conns[cidx].src_outport = cidx; // TODO: CHECK THIS
    conns[cidx].dst_inport = current_switch_link->dest_inport;
    conns[cidx].link_type = current_switch_link->type;
    cidx++;
  }
 }  

 void flexfly_topology::minimal_route_to_switch(switch_id current_sw_addr, 
 												switch_id dest_sw_addr, 
 												routable::path& path) const {
  std::cout << "minimal_route_to_switch?" << std::endl;
 	if (!valid_switch_id(current_sw_addr) || !valid_switch_id(dest_sw_addr)) {
 		return;
 	}
 };

 /**
  * checks if a given switch id in the topology is an optical switch or not
  */
 bool flexfly_topology::is_optical_switch(switch_id swid) const {
 	if (!valid_switch_id(swid)) {
 		return false;
 	} 
  int group_num = swid / (switches_per_group_ + num_optical_switches_per_group_);
 	return swid >= group_num * (switches_per_group_ + num_optical_switches_per_group_) + switches_per_group_;
 };

 /**
  * checks if a given switch id in the topology is an electrical switch or not
  */
 bool flexfly_topology::is_electrical_switch(switch_id swid) const {
  return valid_switch_id(swid) && !is_optical_switch(swid);
 };

 /**
  * NOTE: This can wait, DON'T IMPLEMENT FOR NOW
  */
 void flexfly_topology::dfly_wire(std::string& filename) {
 	// TODO: Implement this in the future to configure things from say an input adjacency graph
 	return;
 };

 // DONE (RECHECK)
 void flexfly_topology::configure_optical_or_electrical_port_params(switch_id swid, 
                                                                    std::string& str, 
                                                                    sprockit::sim_parameters* switch_params) const {
 	if (!switch_params) {
 		//spkt_throw_printf("switch_link params fed in is a null pointer, switch_link params not defined?");
 		return;
 	}
 	
 	sprockit::sim_parameters* specific_switch_params = switch_params->get_namespace(str); // refers to either optical or electrical
  sprockit::sim_parameters* switch_link_params = specific_switch_params->get_namespace("switch_link");

 	double bandwidth = switch_link_params->get_bandwidth_param("bandwidth"); // in units of bytes/sec I think
 	int port_count = switch_link_params->get_int_param("port_count");
 	long buf_space = switch_link_params->get_byte_length_param("buffer_size");
 	int switch_link_redundancy = switch_link_params->get_optional_int_param("switch_link_redundancy", 1);
 	int credits = ((int) buf_space)*switch_link_redundancy;
 	for (int i = 0; i < port_count; i++) {
 		topology::setup_port_params(i, credits, bandwidth, switch_link_params, switch_params);
 	}
 };

 // DONE
 void flexfly_topology::configure_individual_port_params(switch_id src,
          												 sprockit::sim_parameters* switch_params) const {
 	std::cout << "configure_individual_port_params" << std::endl;
  if (!switch_params) {
 		//spkt_throw_printf("switch_params is null");
 	}

 	//sim_params* switch_link_params = switch_params->get_namespace("switch_link");
 	//sim_params* switch_params = switch_params->get_namespace("switch_link");
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

 /**
  * @Brief Given a source switch and a destination switch, connects the source switch with the 
  * dest switch
  * NOTE: This member function should form a bidirectional switch_link
  */
  void flexfly_topology::connect_switches(switch_id source, switch_id dest, Link_Type ltype) {
  //std::string msg = (ltype == Link_Type::electrical) ? "electrical" : "optical";
  //std::cout << "connect switches " << std::to_string(source) << " and " << std::to_string(dest) 
  //          << " using " << msg << std::endl;
 	if (switch_connection_map_.count(source) == 0) {
    //std::cout << "INSERTION FOR SWITCH_ID = " << std::to_string(source) << std::endl;
    switch_connection_map_[source] = std::vector<switch_link*>();
 	}
  if (switch_connection_map_.count(dest) == 0) {
    //std::cout << "INSERTION FOR SWITCH_ID = " << std::to_string(dest) << std::endl;
    switch_connection_map_[dest] = std::vector<switch_link*>();
  }
 	std::unordered_map<switch_id, std::vector<switch_link*>>::const_iterator tmp = switch_connection_map_.find(source);
  const std::vector<switch_link*>& source_switch_connection_vector = tmp->second;
  tmp = switch_connection_map_.find(dest);
  const std::vector<switch_link*>& dest_switch_connection_vector = tmp->second;
  int src_outport = source_switch_connection_vector.size();
  int dest_inport = dest_switch_connection_vector.size();
  switch_link* src_switch_link = (switch_link *) malloc(sizeof(switch_link));
  src_switch_link->dest_sid = dest;
  src_switch_link->dest_inport = dest_inport;
  src_switch_link->type = ltype;
 }

 bool flexfly_topology::switch_id_slot_filled(switch_id sid) const {
  return (sid < max_switch_id_);
 }


 // TODO: FIGURE OUT HOW THIS FUNCTION WORKS
 //switch_id flexfly_topology::netlink_to_injection_switch(netlink_id nodeaddr, uint16_t& switch_port) const {
  //return 0;
 //};

 // TODO: FIGURE OUT HOW THIS FUNCTION WORKS
 //switch_id flexfly_topology::netlink_to_ejection_switch(netlink_id nodeaddr, uint16_t& switch_port) const {
  //return 0;
 //};

 void flexfly_topology::configure_vc_routing(std::map<routing::algorithm_t, int>& m) const {
  m.insert({routing::minimal, 3});
  m.insert({routing::minimal_adaptive, 3});
  m.insert({routing::valiant, 3});
  m.insert({routing::ugal, 3});
  return;
 };

  switch_id flexfly_topology::node_to_ejection_switch(node_id addr, uint16_t& port) const {
    std::cout << "node_to_ejection_switch?" << std::endl;
    switch_id swid = addr / nodes_per_switch_; // this gives us the switch id of the switch node addr is connected to
    std::unordered_map<switch_id, std::vector<switch_link*>>::const_iterator tmp_iter = switch_connection_map_.find(swid);
    const std::vector<switch_link*>& conn_vector = tmp_iter->second;
    port = std::max((int) (conn_vector.size() - 1), 0) + ((int) swid) * nodes_per_switch_; // CHECK THIS AGAIN
    return swid;
  };
  
  switch_id flexfly_topology::node_to_injection_switch(node_id addr, uint16_t& port) const {
    std::cout << "node_to_injection_switch?" << std::endl;
    return node_to_ejection_switch(addr, port);
  };

  int flexfly_topology::minimal_distance(switch_id src, switch_id dst) const {
    std::cout << "src switch id: " << std::to_string(src) << " dst switch id: " << std::to_string(dst) << std::endl;
    if (src == dst) { // same switch
      return 0;
    } else if ((src / switches_per_group_) == (dst / switches_per_group_)) { // same group
      return 1;
    } else { // different group but can reach either by 1 global and 1 local or 1 local and then 1 global
      std::unordered_map<switch_id, std::vector<switch_link*>>::const_iterator tmp_iter = switch_connection_map_.find(src);
      const std::vector<switch_link*>& conn_vector = tmp_iter->second;
      int dest_group = dst / (switches_per_group_ + num_optical_switches_per_group_);
      bool two_or_three = true; 
      // 1) have to search through the vector of all port connections of it's own global link
      // 2) also to search through the connection vectors of all of the switch's neighbors
      // 3) at the same time, you'd need to be aware of what the optical switches' internal states are
      //    and if they are connecting the groups together. 
      for (switch_link* tmp_link : conn_vector ) {
        if (tmp_link->type == electrical)
          continue;
        if (tmp_link->dest_sid / (switches_per_group_ + num_optical_switches_per_group_) == dest_group)
          return 2;
      }

      //if (conn_vector)
      //dist = 2;
      return two_or_three ? 2 : 3;
    }
  };

  int flexfly_topology::num_hops_to_node(node_id src, node_id dst) const {
    //std::cout << "num_hops_to_node?" << std::endl;
    int supposed_src_swid = src / (nodes_per_switch_);
    int supposed_dst_swid = dst / (nodes_per_switch_);
    int src_group = supposed_src_swid / switches_per_group_;
    int dst_group = supposed_dst_swid / switches_per_group_;
    switch_id actual_src_swid = src_group * (switches_per_group_ + num_optical_switches_per_group_) + supposed_src_swid % switches_per_group_;
    switch_id actual_dst_swid = dst_group * (switches_per_group_ + num_optical_switches_per_group_) + supposed_dst_swid % switches_per_group_;
    int min_dist = minimal_distance(actual_dst_swid, actual_dst_swid);
    //std::cout << std::to_string(min_dist) << std::endl;
    return min_dist + 2; // added by 2 because each node is 1 hop away from it's switch
  };

  void flexfly_topology::nodes_connected_to_injection_switch(switch_id swid, 
                                                              std::vector<injection_port>& nodes) const {
    std::cout << "nodes_connected_to_injection_switch?" << std::endl;
    int i = 0;
    switch_id private_swid = public_swid_to_private_swid(swid);
    
    for (int i = 0; i < nodes_per_switch_; i++) {
      nodes[i].nid = private_swid * nodes_per_switch_ + i;
      std::unordered_map<switch_id, std::vector<switch_link*>>::const_iterator tmp_iter = switch_connection_map_.find(swid);
      const std::vector<switch_link*>& switch_conn_vector = tmp_iter->second;
      nodes[i].port = (switch_conn_vector.size()) + i;
    }

  };

  void flexfly_topology::nodes_connected_to_ejection_switch(switch_id swid, 
                                                              std::vector<injection_port>& nodes) const { 
    std::cout << "nodes_connected_to_ejection_switch?" << std::endl;
    flexfly_topology::nodes_connected_to_injection_switch(swid, nodes);
  };

  /*
  void flexfly_topology::minimal_route_to_switch(switch_id current_sw_addr, 
                                                  switch_id dest_sw_addr, 
                                                  routable::path& path) const {
    
  };
  */
  /**
   * @Bool returns boolean true when two groups are linked via optical switch
   * false otherwise
   * THIS IS A DUMMY VERSION FOR NOW AND NEEDS TO BE IMPLEMENTED IN ORDER FOR IT TO WORK
   */
  bool flexfly_topology::is_group_connected(int src_gid, int dst_gid) const {
    bool res = true;
    return res;
  };

  inline switch_id flexfly_topology::public_swid_to_private_swid(switch_id swid) const {
    switch_id offset = swid % (switches_per_group_ + num_optical_switches_per_group_);
    switch_id group = swid / (switches_per_group_ + num_optical_switches_per_group_);
    return group + offset;
  };

  // returns the group id of a given switch
  inline int flexfly_topology::group_from_swid(switch_id swid) const {
    return swid / (switches_per_group_ + num_optical_switches_per_group_);
  };

  /**
   * Used to print the connectivity matrix of the entire topology 
   * Doesn't need to take in any sort of arguments and does not return any
   * argument
   */
  void flexfly_topology::print_topology() const {
    for (std::pair<switch_id, std::vector<switch_link*>> key_val_pair : 
                switch_connection_map_ ) {
      switch_id swid = key_val_pair.first;
      print_port_connection_for_switch(swid);
    }
  };

  /**
   * prints out the all the connections for each switch
   */
  void flexfly_topology::print_port_connection_for_switch(switch_id swid) const {
    std::unordered_map<switch_id, std::vector<switch_link*>>::const_iterator tmp_iter = switch_connection_map_.find(swid);
    if (tmp_iter == switch_connection_map_.end()) {
      std::printf("nothing to print, this switch with swid: %d does not exist\n", (int) swid);
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
        //spkt_throw_printf(sprockit::value_error, "A switch link with swid: " +
        //                 std::to_string(swid) + " is null, which it shouldn't be");
      }
      i++;
    }
    std::cout << message.str(); 
  };

  void flexfly_topology::print_connectivity_matrix() const {

  };

  /**
   * @brief num_endpoints To be distinguished slightly from nodes.
   * Multiple nodes can be grouped together with a netlink.  The netlink
   * is then the network endpoint that injects to the switch topology
   * @return
   */
  int flexfly_topology::num_netlinks() const {
    std::cout << "num_netlinks?" << std::endl;
    return 4;
  }; 

  switch_id flexfly_topology::max_netlink_id() const {
    std::cout << "max_netlink_id?" << std::endl;
    return max_switch_id_;
  };

  bool flexfly_topology::netlink_id_slot_filled(node_id nid) const {
    std::cout << "netlink_id_slot_filled?" << std::endl;
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
  switch_id flexfly_topology::netlink_to_injection_switch(
        netlink_id nodeaddr, uint16_t& switch_port) const {
    std::cout << "netlink_to_injection_switch?" << std::endl;
    return max_switch_id_;
  };

  /**
     For a given node, determine the ejection switch
     All messages to this node eject into the network
     through this switch
     @param nodeaddr The node to eject from
     @param switch_port [inout] The port on the switch the node ejects on
     @return The switch that ejects into the node
  */
  switch_id flexfly_topology::netlink_to_ejection_switch(
        netlink_id nodeaddr, uint16_t& switch_port) const {
    std::cout << "netlink_to_ejection_switch?" << std::endl;
    return max_switch_id_;
  };

  
  bool flexfly_topology::node_to_netlink(node_id nid, node_id& net_id, int& offset) const {
    std::cout << "node_to_netlink?" << std::endl;
    return true;
  };

  switch_id flexfly_topology::node_to_logp_switch(node_id nid) const {
    switch_id swid = nid / (switches_per_group_);
    return swid;
  };
}
}

