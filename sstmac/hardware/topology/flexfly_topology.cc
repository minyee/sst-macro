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


namespace sst{
namespace hw{
 
 void flexfly_topology::setup_flexfly_topology() {
 	// first iterate over all the groups
 	switch_id swid = 0;

 	//setup the electrical links
 	for (int group = 0; group < num_groups_; group++) {
 		for (int intra_group_index = 0; intra_group_index < switches_per_group_; intra_group_index++) {
 			for (int other_intra_group_indices = swid - group * switches_per_group_ + 1; 
 					other_intra_group_indices < switches_per_group_; 
 					other_intra_group_indices++) {
 				switch_id dest_id = swid + other_intra_group_indices;
 				connect_switches(swid, dest_id, Link_Type::electrical); // form all intra-group electrical connections first
        // NOTE that at this point, we are assuming that intra-group topology is all to all
 			}
 			swid++;
 		}

    for (int i = 0; i < switches_per_group_; i++) {
      switch_id elec_switch_id = swid - 1 
      connect_switches()
    }
    // assume that there is 1 optical switch per group
    swid++;

 	}

  // setup the optical links
  

  

  // setup the optical switches
  max_switch_id_ = swid; // REMEMBER TO SET THE MAXIMUM SWITCH ID
 };

 flexfly_topology::flexfly_topology(sprockit::sim_parameters* params) {

 	// initialization of the private class member variables 
 	// intra_group_diameter_ = 1; // this assumes that intra group topology is all to all
 	num_groups_ = get_optional_int_param("groups", 7);
 	num_optical_switches_per_group_ = 1; // This can be changed
 	switches_per_group_ = get_optional_int_param("switches_per_group", 6);
 	nodes_per_switch_ = get_optional_int_param("nodes_per_switch", 7);

 	num_optical_switches_ = num_groups_ * num_optical_switches_per_group_; // for now, assume that each group will have one optical switch attached to itself.

 	num_total_switches_ = num_optical_switches_ + num_groups_ * switches_per_group_;
 	switch_id sid = 0;
 	setup_flexfly_topology();
 };

 // need to deallocate everything at the deconstructor
 flexfly_topology::~flexfly_topology() {
  for (std::vector<link> connection_vector : switch_connection_map_) {
    for (link* link_ptr : connection_vector) {
      free(link_ptr);
    }
  }
 };

 void flexfly_topology::config_metis(metis_configuration* configuration) {
	if (!configuration) {
		return;
	}
	// if reach this point, this means that the configuration pointer is not null

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
 }
 
 // DONE
 void flexfly_topology::connected_outports(switch_id src, 
 											std::vector<topology::connection>& conns) {
 	if (!valid_switch_id(src) || switch_connection_map_.count(src) == 0) {
 		return;
 	}

  int cidx = 0;
  std::vector<link*> link_vectors = switch_connection_map_.find(src)
  assert(link_vectors != switch_connection_map_.end());
  for (link* current_link : link_vectors) {
    conns[cidx].src = src;
    conns[cidx].dst = current_link->dest_sid;
    conns[cidx].src_outport = cidx; // TODO: CHECK THIS
    conns[cidx].dst_inport = current_link->dest_inport;
    conns[cidx].link_type = current_link->type;
    cidx++;
  }
 	return;
 }

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
 	
 };

 /**
  * checks if a given switch id in the topology is an electrical switch or not
  */
 bool flexfly_topology::is_electrical_switch(switch_id swid) {
 	if (!valid_switch_id(swid)) {
 		return false;
 	} 
 };

 /**
  * NOTE: This can wait, DON'T IMPLEMENT FOR NOW
  */
 void flexfly_topology::wire(std::string& filename) {
 	// TODO: Implement this in the future to configure things from say an input adjacency graph
 	return;
 }

 // DONE (RECHECK)
 void flexfly_topology::configure_optical_or_electrical_port_params(switch_id swid, const std::string& str, sprockit::sim_parameters* switch_params) {
 	if (!link_params) {
 		spkt_throw_printf("link params fed in is a null pointer, link params not defined?");
 		return;
 	}
 	
 	sim_params* specific_switch_params = switch_params->get_namespace(str); // refers to either optical or electrical
 	int port_count = specific_switch_params->
 	sim_params* link_params = specific_switch_params->get_namespace("link");

 	double bandwidth = link_params->get_bandwidth_param("bandwidth"); // in units of bytes/sec I think
 	int port_count = link_params->get_int_param("port_count");
 	long buf_space = link_params->get_byte_length_param("buffer_size");
 	int link_redundancy = get_optional_int_param("link_redundancy", 1);
 	int credits = ((int) buf_space)*link_redundancy;
 	for (int i = 0; i < port_count; i++) {
 		topology::setup_port_params(i, credits, bandwidth, link_params, switch_params);
 	}
 }

 // DONE
 void flexfly_topology::configure_individual_port_params(switch_id src,
          												 sprockit::sim_parameters* switch_params) const {
 	if (!switch_params) {
 		spkt_throw_printf("switch_params is null");
 	}

 	//sim_params* link_params = switch_params->get_namespace("link");
 	//sim_params* switch_params = switch_params->get_namespace("link");
 	if (valid_switch_id(src)) {
 		std::string str;
 		if (is_optical_switch(src)) {
 			str = "optical";
 		} else {
 			str = "electrical"
 		}
 		configure_optical_or_electrical_port_params(src, (const std::string&) str, switch_params);
 	} 	
 }

 // DONE
 inline bool flexfly_topology::valid_switch_id(switch_id swid) {
 	return (id >= 0) && (id < num_optical_switches_ + num_optical_switches_);
 }

 // DONE
 /**
  * @Brief Given a source switch and a destination switch, connects the source switch with the 
  * dest switch
  * NOTE: This member function should form a bidirectional link
  */
 bool flexfly_topology::connect_switches(switch_id source, switch_id dest, Link_Type ltype) {
 	  if (switch_connection_map_.count(source) == 0) {
      switch_connection_map_.insert(source, std::vector<link*>);
 	  }
    if (switch_connection_map_.count(dest) == 0) {
      switch_connection_map_.insert(dest, std::vector<link*>);
    }
    bool connection_successful = true;
 	  std::vector<link*> source_switch_connection_vector = switch_connection_map_.find(source);
    std::vector<link*> dest_switch_connection_vector = switch_connection_map_.find(dest);
    int src_outport = source_switch_connection_vector.size();
    int dest_inport = dest_switch_connection_vector.size();

    link* src_link = (link *) malloc(sizeof(link));
    link* dest_link = (link *) malloc(sizeof(link));
    src_link->dest_sid = dest;
    src_link->dest_inport = dest_inport;
    src_link->type = dest_link->type = ltype;
    dest_link->dest_sid = src;
    dest_link->dest_inport = src_outport;
 }


}
}