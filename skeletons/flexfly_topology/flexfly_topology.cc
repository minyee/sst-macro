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
#include "connectivity_matrix.h"


namespace sstmac {
namespace hw {

 flexfly_topology::flexfly_topology(sprockit::sim_parameters* params) : 
                              structured_topology(params,InitMaxPortsIntra::I_Remembered, 
                                                  InitGeomEjectID::I_Remembered) {
 	num_groups_ = params->get_optional_int_param("groups", 7); // controls g
 	switches_per_group_ = params->get_optional_int_param("switches_per_group", 6); // controls a
  switches_per_group_ = num_groups_ - 1;
 	nodes_per_switch_ = params->get_optional_int_param("nodes_per_switch", 4);
  optical_switch_radix_ = params->get_optional_int_param("optical_switch_radix", switches_per_group_);
 	num_optical_switches_ = switches_per_group_;
  optical_switch_radix_ = num_groups_;
  max_switch_id_ = num_optical_switches_ + (num_groups_ * switches_per_group_) - 1;
  max_node_id_ = num_groups_ * switches_per_group_ * nodes_per_switch_ - 1;
  group_connectivity_matrix_ = (uint32_t **) malloc(num_groups_ * sizeof(uint32_t*));
  for (int i = 0; i < num_groups_; i++) {
    group_connectivity_matrix_[i] = (uint32_t *) calloc(num_groups_, sizeof(uint32_t));
  }
 	setup_flexfly_topology();
  check_intergroup_connection();
  configure_simpler_model(num_groups_, optical_inout_connectivity_); 
  std::cout << "flexfly_topology -> optical_inout_connectivity_ has size: " << std::to_string(optical_inout_connectivity_.size()) << std::endl;
 }

 flexfly_topology::~flexfly_topology() {
   for (const std::pair<switch_id, std::vector<switch_link*>> elem : switch_outport_connection_map_) {
     const std::vector<switch_link*>& conn_vector = elem.second;  
     for (auto const&  switch_link_ptr : conn_vector) {
       free(switch_link_ptr);
     }
   }
   for (int i = 0; i < num_groups_; i++) {
    delete group_connectivity_matrix_[i];
   }
   delete group_connectivity_matrix_;
 }

 /**
  * the implementation is the one where the optical radix is the same as the # of 
  * groups, and there will be switches_per_group_ number of optical switches
  */
 void flexfly_topology::setup_flexfly_topology() {
   // first step: connect all the switches within the same group together
   for (int group = 0; group < num_groups_; group++) {
     switch_id group_offset = group * switches_per_group_;
     for (int index = 0; index < switches_per_group_; index++) {
       switch_id swid = group_offset + index;
       for (int target_index = index + 1; target_index < switches_per_group_; target_index++) {
         switch_id target_swid = group_offset + target_index;
         connect_switches(swid, target_swid, Link_Type::electrical);
         connect_switches(target_swid, swid, Link_Type::electrical);
       }
     }  
   }

   // second step: connect all the switches within groups to the optical switches
   for (int group = 0; group < num_groups_; group++) {
     switch_id group_offset = group * switches_per_group_;
     for (int index = 0; index < switches_per_group_; index++) {
       switch_id swid = group_offset + index;
       switch_id optical_swid = num_groups_ * switches_per_group_ + index;
       connect_switches(swid, optical_swid, Link_Type::optical);
       connect_switches(optical_swid, swid, Link_Type::optical);
     }
   }
 }


 void flexfly_topology::configure_metis(metis_config* configuration) const {
	if (!configuration) {
		return;
	}
	configuration->nvtxs = (idx_t) num_groups_ * switches_per_group_;
 }

 void flexfly_topology::minimal_route_to_switch(switch_id src_switch_addr, 
 												switch_id dst_switch_addr, 
 												routable::path& path) const {
 	if (!valid_switch_id(src_switch_addr) || !valid_switch_id(dst_switch_addr)) {
 		return;
 	}
  int src_group = group_from_swid(src_switch_addr);
  int dst_group = group_from_swid(dst_switch_addr);

  if (src_group == dst_group) {
    std::vector<topology::connection> conns;  
    connected_outports(src_switch_addr, conns);
    for (int i = 0; i < conns.size(); i++) {
      if (conns[i].dst == dst_switch_addr) {
        path.set_outport(i);
        return;
      }
    }
  } //else {
    //path.set_outport();
  //}

 };

 /**
  * checks if a given switch id in the topology is an optical switch or not
  */
 bool flexfly_topology::is_optical_switch(switch_id swid) const {
  if (!valid_switch_id(swid)) {
 		return false;
 	} 
 	return swid >= num_groups_ * switches_per_group_;
 };

 /**
  * checks if a given switch id in the topology is an electrical switch or not
  */
 bool flexfly_topology::is_electrical_switch(switch_id swid) const {
  return valid_switch_id(swid) && !is_optical_switch(swid);
 };

 // DONE (RECHECK)
 void flexfly_topology::configure_optical_or_electrical_port_params(switch_id swid, 
                                                                    std::string& str, 
                                                                    sprockit::sim_parameters* switch_params) const {
 	if (!switch_params) {
 		return;
 	}
 	// refers to either optical or electrical
  sprockit::sim_parameters* link_params = switch_params->get_namespace("link");
 	double bandwidth = switch_params->get_optional_bandwidth_param("bandwidth",1000); // in units of bytes/sec I think
 	long buf_space = switch_params->get_optional_byte_length_param("buffer_size", 1024);
  int switch_link_redundancy = switch_params->get_optional_int_param("switch_link_redundancy", 1);
  int credits = ((int) buf_space)*switch_link_redundancy;
  int port_count = is_optical_switch(swid) ? this->num_groups_ : this->switches_per_group_ + this->nodes_per_switch_;
 	for (int i = 0; i < port_count; i++) {
 		topology::setup_port_params(i, credits, bandwidth, link_params, switch_params);
 	}
 };

 // DONE
 void flexfly_topology::configure_individual_port_params(switch_id src,
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

 /**
  * @Brief Given a source switch and a destination switch, connects the source switch with the 
  * dest switch
  * NOTE: This member function should form a bidirectional switch_link
  */
  void flexfly_topology::connect_switches(switch_id src, switch_id dst, Link_Type ltype) {
    std::vector<switch_link*>& src_outport_connection_vector = switch_outport_connection_map_[src];
    std::vector<switch_link*>& dst_inport_connection_vector = switch_inport_connection_map_[dst];
    switch_link *conns = new switch_link();
    conns->src_sid = src;
    conns->dest_sid = dst;
    conns->dest_inport = dst_inport_connection_vector.size(); 
    conns->src_outport = src_outport_connection_vector.size();
    conns->type = ltype;
    int src_port = src_outport_connection_vector.size();
    src_outport_connection_vector.push_back(conns);
    dst_inport_connection_vector.push_back(conns);
    return;
 }


void flexfly_topology::connected_outports(const switch_id src, 
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


bool flexfly_topology::switch_id_slot_filled(switch_id sid) const {
  return (sid < max_switch_id_);
}

 void flexfly_topology::configure_vc_routing(std::map<routing::algorithm_t, int>& m) const {
  m.insert({routing::minimal, 3});
  m.insert({routing::minimal_adaptive, 3});
  m.insert({routing::valiant, 3});
  m.insert({routing::ugal, 3});
  return;
 };

  switch_id flexfly_topology::node_to_ejection_switch(node_id addr, uint16_t& port) const {
    switch_id swid = addr / nodes_per_switch_; // this gives us the switch id of the switch node addr is connected to
    std::unordered_map<switch_id, std::vector<switch_link*>>::const_iterator tmp_iter = switch_outport_connection_map_.find(swid);
    const std::vector<switch_link*>& conn_vector = tmp_iter->second;
    port = std::max((int) (conn_vector.size() - 1), 0) + ((int) swid) * nodes_per_switch_; // CHECK THIS AGAIN
    return swid;
  };
  
  switch_id flexfly_topology::node_to_injection_switch(node_id addr, uint16_t& port) const {
    return node_to_ejection_switch(addr, port);
  };

  /**
   * NOTE: This method does not include the hop to an optical switch
   **/
  int flexfly_topology::minimal_distance(switch_id src, switch_id dst) const {
    std::cout << "src switch id: " << std::to_string(src) << " dst switch id: " << std::to_string(dst) << std::endl;
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

  int flexfly_topology::num_hops_to_node(node_id src, node_id dst) const {
    int src_swid = src / (nodes_per_switch_);
    int dst_swid = dst / (nodes_per_switch_);
    int min_dist = minimal_distance(src_swid, dst_swid);
    return min_dist + 2; // added by 2 because each node is 1 hop away from it's switch
  };

  void flexfly_topology::nodes_connected_to_injection_switch(switch_id swid, 
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

  void flexfly_topology::nodes_connected_to_ejection_switch(switch_id swid, 
                                                              std::vector<injection_port>& nodes) const { 
    flexfly_topology::nodes_connected_to_injection_switch(swid, nodes);
  };

  // returns the group id of a given switch
  int flexfly_topology::group_from_swid(switch_id swid) const {
    return swid / (switches_per_group_);
  };

  /**
   * Used to print the connectivity matrix of the entire topology 
   * Doesn't need to take in any sort of arguments and does not return any
   * argument
   */
  void flexfly_topology::print_topology() const {
    for (std::pair<switch_id, std::vector<switch_link*>> key_val_pair : 
                switch_outport_connection_map_ ) {
      switch_id swid = key_val_pair.first;
      print_port_connection_for_switch(swid);
    }
  };

  /**
   * prints out the all the connections for each switch
   */
  void flexfly_topology::print_port_connection_for_switch(switch_id swid) const {
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
  int flexfly_topology::num_netlinks() const {
    std::cout << "num_netlinks?" << std::endl;
    return 1;
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
    return true;
  };

  switch_id flexfly_topology::node_to_switch(node_id nid) const {
    switch_id swid = nid / (nodes_per_switch_);
    return swid;
  };

  /**
   * Checks that each in and out port of a given index of all optical switches
   * get connected to the exact same switches.
   **/
  void flexfly_topology::check_intergroup_connection() const {
    for (int i = 0; i < num_optical_switches_; i++) {
      int index = i + num_groups_ * switches_per_group_;
      if (!is_optical_switch(index)) {
        spkt_abort_printf("Not an optical switch");
      }
      std::unordered_map<switch_id, std::vector<switch_link*>>::const_iterator tmp_in_iter = switch_inport_connection_map_.find(index);
      std::unordered_map<switch_id, std::vector<switch_link*>>::const_iterator tmp_out_iter = switch_outport_connection_map_.find(index);
      const std::vector<switch_link*>& conn_in_vector = tmp_in_iter->second;
      const std::vector<switch_link*>& conn_out_vector = tmp_out_iter->second;
      if (conn_in_vector.size() == 0 || conn_out_vector.size() != conn_in_vector.size()) {
        spkt_abort_printf("Size equals zero or non-equal inport count and outport count");
      }
      for (int i = 0; i < conn_in_vector.size(); i++) {
        if (conn_in_vector[i]->src_sid != conn_out_vector[i]->dest_sid) {
          spkt_abort_printf("Optical Switch id: %d has different in-out connectivity\n",index);
        }
        switch_id swid = conn_in_vector[i]->src_sid;
        if (group_from_swid(swid) != i) {
          spkt_abort_printf("Optical Switch id: %d doesn't have properly-connected inout ports to group\n",index);  
        }
      }
    }
  };
}
}

