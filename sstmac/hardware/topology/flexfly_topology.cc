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
#include <sstmac/hardware/topology/flexfly_topology.h>



namespace sstmac {
namespace hw {

 flexfly_topology::flexfly_topology(sprockit::sim_parameters* params) : topology(params) {

 	// initialization of the private class member variables 
 	// intra_group_diameter_ = 1; // this assumes that intra group topology is all to all
 	num_groups_ = params->get_optional_int_param("groups", 7);
 	num_optical_switches_per_group_ = 1; // This can be changed
 	switches_per_group_ = params->get_optional_int_param("switches_per_group", 6);
 	nodes_per_switch_ = params->get_optional_int_param("nodes_per_switch", 7);
  //p_ = get_optional_int_param("");
 	num_optical_switches_ = num_groups_ * num_optical_switches_per_group_; // for now, assume that each group will have one optical switch attached to itself.

 	num_total_switches_ = num_optical_switches_ + num_groups_ * switches_per_group_;
 	sstmac::switch_id sid = 0;
 	setup_flexfly_topology();
 }

 // need to deallocate everything at the deconstructor
 flexfly_topology::~flexfly_topology() {
  for (const std::pair<switch_id, std::vector<switch_link*>> elem : switch_connection_map_) {
    const std::vector<switch_link*>& conn_vector = elem.second;  
    //for (auto it = switch_connection_map_.begin(); it != switch_connection_map_.end(); ++it){  
    for (auto const&  switch_link_ptr : conn_vector) {
      free(switch_link_ptr);
    }
    //conn_vector.clear();
    //conn_vector.shrink_to_fit();
    // delete conn_vector; 
    //const_cast<std::vector<switch_link*>&>(conn_vector);
  }
 }

 void flexfly_topology::setup_flexfly_topology() {
  // first iterate over all the groups
  switch_id swid = 0;

  // SETUP ALL-TO-ALL ELECTRICAL SWITCHES
  // setup the electrical switch_links
  for (int group = 0; group < num_groups_; group++) {
    for (int intra_group_index = 0; intra_group_index < switches_per_group_; intra_group_index++) {
      for (int other_intra_group_indices = swid - group * switches_per_group_ + 1; 
          other_intra_group_indices < switches_per_group_; 
          other_intra_group_indices++) {
        switch_id dest_id = swid + other_intra_group_indices;
        connect_switches(swid, dest_id, Link_Type::electrical); // form all intra-group electrical connections first
        connect_switches(dest_id, swid, Link_Type::electrical);
        // NOTE that at this point, we are assuming that intra-group topology is all to all
      }
      //node_connection_map_.insert(swid, new std::vector<switch_link*>());
      swid++;
    }
    std::vector<switch_link*> tmp = std::vector<switch_link*>();
    switch_connection_map_[swid] = tmp;
    swid++;
  }

  // SETUP OPTICAL SWITCHES
  for (int g = 0; g < num_groups_; g++) {
    switch_id curr_optical_swid = (g + 1) * switches_per_group_;
    for (int gg = 0; gg < num_groups_; gg++) {
      if ((gg + 1) * switches_per_group_ == curr_optical_swid) {
        continue;
      } else {
        for (int a = 0; a < switches_per_group_; a++) {
          switch_id curr_electrical_swid = g * switches_per_group_ + a;
          connect_switches(curr_optical_swid, curr_electrical_swid, Link_Type::optical);
          connect_switches(curr_electrical_swid, curr_optical_swid, Link_Type::optical); 
        }
      }
    } 
  }
  max_switch_id_ = swid; // REMEMBER TO SET THE MAXIMUM SWITCH ID
 }

 void flexfly_topology::configure_metis(metis_config* configuration) const {
	if (!configuration) {
		return;
	}
	configuration->nvtxs = (idx_t) num_groups_ * switches_per_group_;
 }


/*
 int flexfly_topology::minimal_distance(switch_id src, switch_id dest) const {
 	if (src == dest) {
 		return 0;
 	} else if (src / num_groups_ == dest / num_groups_) { // two switches are in the same group

 		// return std::min(intra_group_diameter_, ); // assume intra group topology is all to all
 	} else if (true) { // in different groups 

 	} else {

 	}
  return 3;
 }
 */
 // DONE
 void flexfly_topology::connected_outports(const switch_id src, 
 											                      std::vector<topology::connection>& conns) const {
 	if (!valid_switch_id(src) || switch_connection_map_.count(src) == 0) 
 		return;
  int cidx = 0;
  


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
 	if (!valid_switch_id(current_sw_addr) || !valid_switch_id(dest_sw_addr)) {
 		return;
 	}
 };


 /**
  * divvy the job of populating the nodes reference to each switch.
  */
 /*
 void flexfly_topology::nodes_connected_to_ejection_switch(switch_id sid, std::vector<topology::injection_port>& nodes) const {
 	bool to_ret = valid_switch_id(sid);
  if (!to_ret) {
 		return;
 	}
 };
 */
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

 // DONE
 //inline bool flexfly_topology::valid_switch_id(switch_id swid) const {
 	//return id < (switches_per_group_ * num_groups_ + num_optical_switches_);
 //};

 // DONE
 /**
  * @Brief Given a source switch and a destination switch, connects the source switch with the 
  * dest switch
  * NOTE: This member function should form a bidirectional switch_link
  */
 bool flexfly_topology::connect_switches(switch_id source, switch_id dest, Link_Type ltype) {
 	if (switch_connection_map_.count(source) == 0) {
    //switch_connection_map_.insert(source, new std::vector<switch_link*>);
    switch_connection_map_[source] = std::vector<switch_link*>();
 	}
  if (switch_connection_map_.count(dest) == 0) {
    switch_connection_map_[dest] = std::vector<switch_link*>();
  }
  bool connection_successful = true;
 	std::unordered_map<switch_id, std::vector<switch_link*>>::const_iterator tmp = switch_connection_map_.find(source);
  const std::vector<switch_link*>& source_switch_connection_vector = tmp->second;
  tmp = switch_connection_map_.find(dest);
  const std::vector<switch_link*>& dest_switch_connection_vector = tmp->second;
  int src_outport = source_switch_connection_vector.size();
  int dest_inport = dest_switch_connection_vector.size();
  switch_link* src_switch_link = (switch_link *) malloc(sizeof(switch_link));
  //switch_link* dest_switch_link = (switch_link *) malloc(sizeof(switch_link));
  src_switch_link->dest_sid = dest;
  src_switch_link->dest_inport = dest_inport;
  src_switch_link->type = ltype;
  //dest_switch_link->dest_sid = src;
  //dest_switch_link->dest_inport = src_outport;
  return connection_successful;
 };

 bool flexfly_topology::switch_id_slot_filled(switch_id sid) const {
  return (sid < max_switch_id_);
 };


 // TODO: FIGURE OUT HOW THIS FUNCTION WORKS
 switch_id flexfly_topology::netlink_to_injection_switch(netlink_id nodeaddr, uint16_t& switch_port) const {
  return 0;
 };

 // TODO: FIGURE OUT HOW THIS FUNCTION WORKS
 switch_id flexfly_topology::netlink_to_ejection_switch(netlink_id nodeaddr, uint16_t& switch_port) const {
  return 0;
 };

 void flexfly_topology::configure_vc_routing(std::map<routing::algorithm_t, int>& m) const {
  m.insert({routing::minimal, 3});
  m.insert({routing::minimal_adaptive, 3});
  m.insert({routing::valiant, 3});
  m.insert({routing::ugal, 3});
  return;
 };

  switch_id flexfly_topology::node_to_ejection_switch(node_id addr, uint16_t& port) const {
    switch_id swid = addr / nodes_per_switch_; // this gives us the switch id of the switch node addr is connected to
    std::unordered_map<switch_id, std::vector<switch_link*>>::const_iterator tmp_iter = switch_connection_map_.find(swid);
    const std::vector<switch_link*>& conn_vector = tmp_iter->second;
    port = std::max((int) (conn_vector.size() - 1), 0) + ((int) swid) * nodes_per_switch_; // CHECK THIS AGAIN
    return swid;
  };
  
  switch_id flexfly_topology::node_to_injection_switch(node_id addr, uint16_t& port) const {
    return node_to_ejection_switch(addr, port);
  };

  int flexfly_topology::minimal_distance(switch_id src, switch_id dst) const {
    
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
    int supposed_src_swid = src / (nodes_per_switch_);
    int supposed_dst_swid = dst / (nodes_per_switch_);
    int src_group = supposed_src_swid / switches_per_group_;
    int dst_group = supposed_dst_swid / switches_per_group_;
    switch_id actual_src_swid = src_group * (switches_per_group_ + num_optical_switches_per_group_) + supposed_src_swid % switches_per_group_;
    switch_id actual_dst_swid = dst_group * (switches_per_group_ + num_optical_switches_per_group_) + supposed_dst_swid % switches_per_group_;
    return minimal_distance(actual_src_swid, actual_dst_swid) + 2; // added by 2 because each node is 1 hop away from it's switch
  };

  void flexfly_topology::nodes_connected_to_injection_switch(switch_id swid, 
                                                              std::vector<injection_port>& nodes) const {
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
    nodes_connected_to_injection_switch(swid, nodes);
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
  void print_topology() const {
    for (std::unordered_map<switch_id, std::vector<switch_link*>> ) {
      print_port_connection_for_switch();
    }
  };

  /**
   * prints out the all the connections for each switch
   */
  void print_port_connection_for_switch(switch_id swid) const {
    std::unordered_map<switch_id, std::vector<switch_link*>>::const_iterator tmp_iter = switch_connection_map_.find(swid);
    if (tmp_iter == switch_connection_map_.end()) {
      std::printf("nothing to print, this switch with swid: %d does not exist\n", (int) swid);
      return;
    }
    std::vector<switch_link*>& connection_vector = tmp_iter->second;

    std::stringstream message;
    int i = 0;
    for (switch_link* sl_ptr : connection_vector) {
      // check if null, if null have to throw an error 
      if (sl_ptr) {
        message << "Dest switch_id: " << std::to_string(sl_Ptr->dest_sid);
        message << " Dest inport: " << std::to_string(sl_ptr->dest_inport);
        message << " Link type: ";
        if (sl_ptr->type == Link_Type::electrical) {
          message << "ELECTRICAL" << endl;
        } else {
          message << "OPTICAL" << endl;
        }
      } else {
        spkt_throw_printf(spkt::error, "A switch link with swid: " +
                         std::to_string(swid) + " is null, which it shouldn't be");
      }
      i++;
    }
    cout << message; 
  }

}
}

