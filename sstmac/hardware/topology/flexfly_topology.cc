/**
 * Author: Min Yee Teh
 */
#include <iostream>	
#include <algorithm>
#include <vector>

#include <sstmac/hardware/topology/flexfly_topology.h>
#include <sstmac/hardware/topology/topology.h>
#include <sstmac/hardware/switch/flexfly_electrical_switch.h>
#include <sstmac/hardware/switch/flexfly_optical_switch.h>
#include <sstmac/common/node_address.h>
#include <sprockit/sim_parameters.h>
#include <sprockit/errors.h> 
#include <stdlib.h>
#include <assertion.h>


namespace sst {
namespace hw {
 
 void flexfly_topology::setup_flexfly_topology() {
 	// first iterate over all the groups
 	switch_id swid = 0;

  // SETUP ALL-TO-ALL ELECTRICAL SWITCHES
 	//setup the electrical switch_links
 	for (int group = 0; group < num_groups_; group++) {
 		for (int intra_group_index = 0; intra_group_index < switches_per_group_; intra_group_index++) {
 			for (int other_intra_group_indices = swid - group * switches_per_group_ + 1; 
 					other_intra_group_indices < switches_per_group_; 
 					other_intra_group_indices++) {
 				switch_id dest_id = swid + other_intra_group_indices;
 				connect_switches(swid, dest_id, switch_link_Type::electrical); // form all intra-group electrical connections first
        connect_switches(dest_id, swid, switch_link_Type::electrical);
        // NOTE that at this point, we are assuming that intra-group topology is all to all
 			}
      //node_connection_map_.insert(swid, new std::vector<switch_link*>());
 			swid++;

 		}
    switch_connection_map_.insert(swid, new std::vector<switch_link*>);
    swid++;
 	}

  assert(swid == num_groups_ * switches_per_group_);

  // SETUP OPTICAL SWITCHES
  for (int g = 0; g < num_groups_; g++) {
    switch_id curr_optical_swid = (g + 1) * switches_per_group_;
    for (int gg = 0; gg < num_groups_; gg++) {
      if ((gg + 1) * switches_per_group_ == curr_optical_swid) {
        continue;
      } else {
        for (int a = 0; a < switches_per_group_; a++) {
          switch_id curr_electrical_swid = g * switches_per_group_ + a;
          connect_switches(curr_optical_swid, curr_electrical_swid, switch_link_Type::optical);
          connect_switches(curr_electrical_swid, curr_optical_swid, switch_link_Type::optical); 
        }
      }
    } 
  }
  max_switch_id_ = swid; // REMEMBER TO SET THE MAXIMUM SWITCH ID

  // NO NEED TO SETUP NODES BECAUSE WE CAN EASILY MAP NODES TO SWITCHES

  // setup nodes
  // for (int node = 0; node < switches_per_group_ * num_groups_ * nodes_per_switch_; node++) {
  // switch_id swid = node / nodes_per_switch_;
    
  // }
 };

 flexfly_topology::flexfly_topology(sprockit::sim_parameters* params) {

 	// initialization of the private class member variables 
 	// intra_group_diameter_ = 1; // this assumes that intra group topology is all to all
 	num_groups_ = get_optional_int_param("groups", 7);
 	num_optical_switches_per_group_ = 1; // This can be changed
 	switches_per_group_ = get_optional_int_param("switches_per_group", 6);
 	nodes_per_switch_ = get_optional_int_param("nodes_per_switch", 7);
  //p_ = get_optional_int_param("");
 	num_optical_switches_ = num_groups_ * num_optical_switches_per_group_; // for now, assume that each group will have one optical switch attached to itself.

 	num_total_switches_ = num_optical_switches_ + num_groups_ * switches_per_group_;
 	switch_id sid = 0;
 	setup_flexfly_topology();
 };

 // need to deallocate everything at the deconstructor
 flexfly_topology::~flexfly_topology() {
  for (std::vector<switch_link>& connection_vector : switch_connection_map_) {
    for (switch_link* switch_link_ptr : connection_vector) {
      free(switch_link_ptr);
    }
    delete connection_vector;
  }
 };

 void flexfly_topology::config_metis(metis_configuration* configuration) {
	if (!configuration) {
		return;
	}
	configuration->nvtxs = (idx_t) num_groups_ * switches_per_group_;
 };



 void flexfly_topology::minimal_path(switch_id src, switch_id dest) {
 	if (src == dest) {
 		return 0;
 	} else if (src / groups_ == dest / groups_) { // two switches are in the same group

 		// return std::min(intra_group_diameter_, ); // assume intra group topology is all to all
 	} else if () { // in different groups 

 	} else {

 	}
 };
 
 // DONE
 void flexfly_topology::connected_outports(switch_id src, 
 											std::vector<topology::connection>& conns) {
 	if (!valid_switch_id(src) || switch_connection_map_.count(src) == 0) {
 		return;
 	}
  int cidx = 0;
  std::vector<switch_link*> switch_link_vectors = switch_connection_map_.find(src)
  assert(switch_link_vectors != switch_connection_map_.end());
  for (switch_link* current_switch_link : switch_link_vectors) {
    conns[cidx].src = src;
    conns[cidx].dst = current_switch_link->dest_sid;
    conns[cidx].src_outport = cidx; // TODO: CHECK THIS
    conns[cidx].dst_inport = current_switch_link->dest_inport;
    conns[cidx].switch_link_type = current_switch_link->type;
    cidx++;
  }
 	return;
 };

 void flexfly_topology::minimal_route_to_switch(switch_id current_sw_addr, 
 												switch_id dest_sw_addr, 
 												routable::path& path) {
 	if (!valid_switch_id(current_sw_addr) || !valid_switch_id(dest_sw_addr)) {
 		return;
 	}
 };


 /**
  * divvy the job of populating the nodes reference to each switch.
  */
 void nodes_connected_to_ejection_switch(switch_id swid, std::vector<injection_port>& nodes) {
 	if (!valid_switch_id(swid)) {
 		return;
 	}
 };
 
 /**
  * checks if a given switch id in the topology is an optical switch or not
  */
 bool flexfly_topology::is_optical_switch(switch_id swid) {
 	if (!valid_switch_id(swid)) {
 		return false;
 	} 
  int group_num = swid / (switches_per_group_ + num_optical_switches_per_group_);
 	return swid >= group_num * (switches_per_group_ + num_optical_switches_per_group_) + switches_per_group_;
 };

 /**
  * checks if a given switch id in the topology is an electrical switch or not
  */
 bool flexfly_topology::is_electrical_switch(switch_id swid) {
  return valid_switch_id(swid) && !is_optical_switch(swid);
 };

 /**
  * NOTE: This can wait, DON'T IMPLEMENT FOR NOW
  */
 void flexfly_topology::wire(std::string& filename) {
 	// TODO: Implement this in the future to configure things from say an input adjacency graph
 	return;
 };

 // DONE (RECHECK)
 void flexfly_topology::configure_optical_or_electrical_port_params(switch_id swid, const std::string& str, sprockit::sim_parameters* switch_params) {
 	if (!switch_link_params) {
 		spkt_throw_printf("switch_link params fed in is a null pointer, switch_link params not defined?");
 		return;
 	}
 	
 	sim_params* specific_switch_params = switch_params->get_namespace(str); // refers to either optical or electrical
 	int port_count = specific_switch_params->
 	sim_params* switch_link_params = specific_switch_params->get_namespace("switch_link");

 	double bandwidth = switch_link_params->get_bandwidth_param("bandwidth"); // in units of bytes/sec I think
 	int port_count = switch_link_params->get_int_param("port_count");
 	long buf_space = switch_link_params->get_byte_length_param("buffer_size");
 	int switch_link_redundancy = get_optional_int_param("switch_link_redundancy", 1);
 	int credits = ((int) buf_space)*switch_link_redundancy;
 	for (int i = 0; i < port_count; i++) {
 		topology::setup_port_params(i, credits, bandwidth, switch_link_params, switch_params);
 	}
 };

 // DONE
 void flexfly_topology::configure_individual_port_params(switch_id src,
          												 sprockit::sim_parameters* switch_params) const {
 	if (!switch_params) {
 		spkt_throw_printf("switch_params is null");
 	}

 	//sim_params* switch_link_params = switch_params->get_namespace("switch_link");
 	//sim_params* switch_params = switch_params->get_namespace("switch_link");
 	if (valid_switch_id(src)) {
 		std::string str;
 		if (is_optical_switch(src)) {
 			str = "optical";
 		} else {
 			str = "electrical"
 		}
 		configure_optical_or_electrical_port_params(src, (const std::string&) str, switch_params);
 	} 	
 };

 // DONE
 inline bool flexfly_topology::valid_switch_id(switch_id swid) {
 	return id < (switches_per_group_ * num_groups_ + num_optical_switches_);
 };

 // DONE
 /**
  * @Brief Given a source switch and a destination switch, connects the source switch with the 
  * dest switch
  * NOTE: This member function should form a bidirectional switch_link
  */
 bool flexfly_topology::connect_switches(switch_id source, switch_id dest, switch_link_Type ltype) {
 	if (switch_connection_map_.count(source) == 0) {
    switch_connection_map_.insert(source, new std::vector<switch_link*>);
 	}
  if (switch_connection_map_.count(dest) == 0) {
    switch_connection_map_.insert(dest, new std::vector<switch_link*>);
  }
  bool connection_successful = true;
 	std::vector<switch_link*>& source_switch_connection_vector = switch_connection_map_.find(source);
  //std::vector<switch_link*> dest_switch_connection_vector = switch_connection_map_.find(dest);
  int src_outport = source_switch_connection_vector->size();
  //int dest_inport = dest_switch_connection_vector.size();
  switch_link* src_switch_link = (switch_link *) malloc(sizeof(switch_link));
  //switch_link* dest_switch_link = (switch_link *) malloc(sizeof(switch_link));
  src_switch_link->dest_sid = dest;
  src_switch_link->dest_inport = dest_inport;
  src_switch_link->type = ltype;
  //dest_switch_link->dest_sid = src;
  //dest_switch_link->dest_inport = src_outport;
  return connection_successful;
 };

 bool flexfly_topology::switch_id_slot_filled(switch_id sid) {
  return (sid < max_switch_id_);
 };


 // TODO: FIGURE OUT HOW THIS FUNCTION WORKS
 switch_id flexfly_topology::netswitch_link_to_injection_switch(netswitch_link_id nodeaddr, uint16_t& switch_port) {
  return 0;
 };

 // TODO: FIGURE OUT HOW THIS FUNCTION WORKS
 switch_id flexfly_topology::netswitch_link_to_ejection_switch(netswitch_link_id nodeaddr, uint16_t& switch_port) {
  return 0;
 };

 void flexfly_topology::configure_vc_routing(std::map<routing::algorithm_t, int>& m) override {
  m.insert(routing::minimal, 3);
  m.insert(routing::minimal_adaptive, 3);
  m.insert(routing::valiant, 3);
  m.insert(routing::ugal, 3);
  return;
 };

  switch_id flexfly_topology::node_to_ejection_switch(node_id addr, uint16_t& port) const override {
    switch_id swid = addr / nodes_per_switch_; // this gives us the switch id of the switch node addr is connected to
    std::vector<link*>& conn_vector = switch_connection_map_.find(swid);
    port = std::max((conn_vector.size() - 1), 0) + ((int) swid) * nodes_per_switch_; // CHECK THIS AGAIN
    return swid;
  };
  
  switch_id flexfly_topology::node_to_injection_switch(node_id addr, uint16_t& port) const override {
    return node_to_ejection_switch(addr, port);
  };

  int flexfly_topology::minimal_distance(switch_id src, switch_id dst) const {
    
    if (src == dst) { // same switch
      return 0;
    } else if ((src / switches_per_group_) == (dst / switches_per_group_)) { // same group
      return 1;
    } else { // different group but can reach either by 1 global and 1 local or 1 local and then 1 global
      std::vector<link*>& conn_vector = switch_connection_map_.find(src);
      int dest_group = dst / (switches_per_group_ + num_optical_switches_per_group_);
      bool two_or_three = true; 
      // 1) have to search through the vector of all port connections of it's own global link
      // 2) also to search through the connection vectors of all of the switch's neighbors
      // 3) at the same time, you'd need to be aware of what the optical switches' internal states are
      //    and if they are connecting the groups together. 
      for (link* tmp_link : conn_vector ) {
        if (tmp_link->type == electrical)
          continue;
        if (tmp_link->dest_sid / (switches_per_group_ + num_optical_switches_per_group_) == dest_group)
          return 2;
      }


      if (conn_vector)
      dist = 2;
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
    
  };

  void flexfly_topology::nodes_connected_to_injection_switch(switch_id swid, 
                                                              std::vector<injection_port>& nodes) const { 
  };

  void flexfly_topology::minimal_route_to_switch(switch_id current_sw_addr, 
                                                  switch_id dest_sw_addr, 
                                                  routable::path& path) const {
    
  };

  bool flexfly_topology::is_group_connected() const {
    
  }
}

}

