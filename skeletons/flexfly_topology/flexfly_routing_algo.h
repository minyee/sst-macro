#include <vector>
#include <stack>
#include <queue>
#include <cassert>
#include <utility>
#include <climits>
#include <cstdlib>
#include "flexfly_topology.h"

// Use stack and returns the distance from src_switch to dst_switch
int depth_first_search(std::vector< std::vector<int> >& connectivity_matrix, 
														int src_switch, 
														int dst_switch) {
	int curr_switch = src_switch;
	std::stack<int> switches_stack;
	switches_stack.push(src_switch);
	std::unordered_map<int, int> distance_switches;
	distance_switches.insert({curr_switch,0});
	bool found = false;
	while(!found && !switches_stack.empty()) {
		curr_switch = switches_stack.top();
		switches_stack.pop();
		auto dist_iter_curr_switch = distance_switches.find(curr_switch);
		int distance_to_curr_switch = dist_iter_curr_switch->second;
		std::vector<int>& connectivity_vector = connectivity_matrix[curr_switch];
		for (int i = 0; i < connectivity_vector.size(); i++) {
			int target_switch = connectivity_vector[i];
			auto searched_node = distance_switches.find(target_switch);
			// if node has never been visited before
			if (searched_node != distance_switches.end()) {
				int distance_to_target_switch = searched_node->second;
				searched_node->second = std::min(distance_to_curr_switch + 1, distance_to_target_switch);
			} else {
				// push node into stack if 
				switches_stack.push(target_switch);
				distance_switches.insert({target_switch, distance_to_curr_switch + 1});
			}
		}
	}
	auto dist_iter_dst_switch = distance_switches.find(dst_switch);
	assert(dist_iter_dst_switch != distance_switches.end());
	return dist_iter_dst_switch->second;
}; 

// use queue and returns the distance from src_switch to dst_switch
int breadth_first_search(std::vector< std::vector<int> >& connectivity_matrix,
														int src_switch, 
														int dst_switch) {
	int curr_switch = src_switch;
	std::queue<int> switches_queue;
	switches_queue.push(src_switch);
	std::unordered_map<int, int> distance_switches;
	distance_switches.insert({curr_switch,0}) ;
	bool found = false;
	while(!found && !switches_queue.empty()) {
		curr_switch = switches_queue.front();
		switches_queue.pop();
		auto dist_iter_curr_switch = distance_switches.find(curr_switch);
		int distance_to_curr_switch = dist_iter_curr_switch->second;
		std::vector<int>& connectivity_vector = connectivity_matrix[curr_switch];
		for (int i = 0; i < connectivity_vector.size(); i++) {
			int target_switch = connectivity_vector[i];
			auto searched_node = distance_switches.find(target_switch);
			// if node has never been visited before
			if (searched_node != distance_switches.end()) {
				int distance_to_target_switch = searched_node->second;
				searched_node->second = std::min(distance_to_curr_switch + 1, distance_to_target_switch);
			} else {
				// push node into stack if 
				switches_queue.push(target_switch);
				distance_switches.insert({target_switch, distance_to_curr_switch + 1});
			}
		}
	}
	auto dist_iter_dst_switch = distance_switches.find(dst_switch);
	assert(dist_iter_dst_switch != distance_switches.end());
	return dist_iter_dst_switch->second;
};

struct distance_optical_id_pair {
	int distance;
	int optical_switch; // through which switch
}

/**
 * Bellman ford routing scheme to routing entire topology 
 * physical_connection is the matrix with entry i,j meaning the number of links connecting switch i
 * to switch j.
 */
void bf_routing_minimal(int src, int dst, 
							std::vector<std::vector<int>>& topology_physical_connection,
							std::vector<std::vector<int>>& optical_inout_configuration) {
	int total_switches = topology_physical_connection.size();
	// START WITH SOME PREPROCESSING, CHANGING THE 
	// inout port connection to a topology adjacency matrix
	std::vector<std::vector<int>> adjacency_matrix;
	adjacency_matrix.resize(total_switches); 
	for (int j = 0; j < total_switches; j++) {
		adjacency_matrix[j].resize(total_switches);
	}
	for (int i = 0; i < total_switches; i++) {
		std::vector<int>& curr_switch_outgoing_links = topology_physical_connection[i];
		for (int j = 0; i < total_switches; j++) {
			adjacency_matrix[i][j]++;
		}
	}
	// DONE WITH PREPROCESSING
	bool converged = false;
	std::vector<std::vector<distance_optical_id_pair>> distance_matrix;

	// initialize the distance matrix
	for (int i = 0; i < total_switches; i++) {
		for (int j = 0; i < total_switches; j) {
			if (adjacency_matrix[i][j] > 0) {
				distance_matrix[i][j].distance = 1;
				//distance_vector[i][j].optical_switch = -1;	
			} else {
				distance_matrix[i][j] = INT_MAX;
			}
			distance_matrix[i][j].optical_switch = -1;
		}
	}

	// start
	while (!converged) {
		// NOTE: i - src_switch, j - dst_switch
		// first iterate through all the switches
		bool changed = false;
		for (int i = 0; i < total_switches; i++) { // first toggle through all the switches
			bool is_curr_switch_optical = flexfly_topology::is_optical_switch(i);
			for (auto neighbor : topology_physical_connection[i]){
				for (int j = 0; j < total_switches; j++) {
					bool is_target_switch_optical = flexfly_topology::is_optical_switch(j);
					distance_matrix[i][j] = min(distance_matrix[i][j], );
				}
			}
		}

		converged = !changed;
	}
}

/**
 * Create 
 **/
void dijkstra_minimal_route(int src, 
				const std::vector<std::vector<switch_link *>> &topology_physical_connection_outport,
				const std::vector<std::vector<switch_link *>> &topology_physical_connection_inport,
				std::vector<int> &distance_vector, // should this be distance vector instead?
				std::vector< flexfly_path * > &path_collection,
				std::vector< std::vector<int> > &optical_inout_configuration) {
	
	int total_switches = topology_physical_connection.size();
	
	std::unordered_map<int,bool> visited_nodes;
	for (int i = 0; i < total_siwtches; i++) {
		visited_nodes[i] = false;
	}
	std::stack<int> switch_stack;
	switch_stack.push(src);
	int curr_switch;

	while (!switch_stack.empty()) {
		/*
		 * First pop the stack
		 */
		curr_switch = switch_stack.top();
		switch_stack.pop();
		/*
		 * First pop the stack
		 */

		std::vector<int> outports;
		std::vector<int> switch_ids;
		
		switch_ids.insert(curr_switch);
		outports.insert(0);

		for (int i = 0; i < topology_physical_connection_outport[curr_switch].size(); i++) {
			outports[0] = i;
			// CASE 1: neighbor happens to be an Optical Switch
			if (is_optical_switch(topology_physical_connection_outport[curr_switch][i]->dest_sid)) {
				int optical_switch = topology_physical_connection_outport[curr_switch][i]->dest_sid;
				std::vector<int>& incoming_links = topology_physical_connection_outport[optical_switch];
				int optical_switch_radix = incoming_links.size();

				for (int optical_switch_inport = 0; optical_switch_inport < optical_switch_radix; optical_switch_inport++) {
					switch_link *curr_incoming_link = incoming_links[optical_switch_inport];
					int src_switch = curr_incoming_link->src_sid;
					if (src_switch == curr_switch) {
						optical_switch_inport = optical_switch_inport;
						break;
					}
				}
				
				int optical_switch_outport = optical_inout_configuration[optical_switch][optical_switch_inport];
				switch_link* outgoing_link = topology_physical_connection_outport[optical_switch][optica_switch_outport];
				assert(outgoing_link->src_sid == optical_switch); // the src switch of this outgoing link has to be the same 
																	// as the id of the optical switch
				int dst_elec_switch = outgoing_link->dest_sid;
				//for (int j = 0; j < optical_switch_radix; j++) {
					// one distance is due to optical switch
				
				//outports
				if (distance_vector[curr_switch] + 2 < distance_vector[dst_elec_switch]) {
					outports.insert(optical_switch_outport);
					switch_ids.insert(optical_switch);
					distance_vector[dst_elec_switch] = distance_vector[curr_switch] + 2;
					clear_path_collections(path_collection, dst_elec_switch);
					add_to_path(path_collection, dst_elec_switch, path_collection[curr_switch], outports, switch_ids, false);
				} else if (distance_vector[curr_switch] + 2 == distance_vector[dst_elec_switch]) {
					add_to_path(path_collection, dst_elec_switch, path_collection[curr_switch], outports, switch_ids, true);
				} 
				//NOTE: Don't push the optical switch into the not visited stack because they work differently.

			} else {
				int dst_elec_switch = topology_physical_connection_outport[curr_switch][i]->dest_sid;
				// CASE 2: neighbor is an Electrical Switch
				
				if (distance_vector[curr_switch] + 1 < distance_vector[dst_elec_switch]) {
					distance_vector[dst_elec_switch] = distance_vector[curr_switch] + 1;
					clear_path_collections(path_collection, dst_elec_switch);
					add_to_path(path_collection, dst_elec_switch, path_collection[curr_switch], outports, switch_ids, false);
				} else if (distance_vector[dst_elec_switch] == distance_vector[curr_switch] + 1) {
					add_to_path(path_collection, dst_elec_switch, path_collection[curr_switch], outports, switch_ids, true);
				}

				// if the neighbor is not optical switch and have yet been visited, then push it into stack
				if (!visited_nodes[i])
					switches_stack.push(topology_physical_connection_outport[curr_switch][i]);
			}
		} 
		visited_nodes[curr_switch] = true;
	}	
};


/**
 * This is a subroutine called by the dijkstra's routing algorithm and not called
 * by anything else.
 * TODO: Finish up add path tonight
 **/
void add_path(std::vector<flexfly_path *> &path_collection, 
				int dst_switch, 
				flexfly_path* basis,
				std::vector<int> &outports,
				std::vector<int> &switch_ids,
				bool useRand) {
	int rand_num = std::srand() % 2;

	assert(outports.size() == switch_ids.size());

	if (!path_collection[dst_switch]) {
		flexfly_path* new_path = new flexfly_path;
		new_path->path_length = outports.size();
		if (basis) {
			new_path->path_length += basis->path_length;
			new_path->path.resize(new_path->path_length);
			for (int i = 0; i < new_path->path_length; i++) {
				if (i < basis->path.size()) {
					new_path->path[i] = basis->path[i];
				} else {
					switch_port_pair *spp = new switch_port_pair();
					spp->switch_id = switch_ids[i - basis->path_length];
					spp->outport = outports[i - basis->path_length];
				}
			}
		} else {
			for (int i = 0; i < new_path->path_length; i++) {
				switch_port_pair *spp = new switch_port_pair();
				spp->switch_id = switch_ids[i - basis->path_length];
				spp->outport = outports[i - basis->path_length];
			}
		}
	} else {
		assert(basis);
		if (!useRand && rand_num == 0) {
			// don't do anything in this case
			return;
		} 
		flexfly_path* new_path = new flexfly_path;
		new_path->path_length = basis->path_length + outports.size();
		new_path->path.resize(new_path->path_length);
		for (int i = 0; i < new_path->path_length; i++) {
			if (i < basis->path.size()) {
				new_path->path[i] = basis->path[i];
			} else {
				switch_port_pair *spp = new switch_port_pair();
				spp->switch_id = switch_ids[i - basis->path_length];
				spp->outport = outports[i - basis->path_length];
			}
		}
	}
	return;
};

/**
 * Deletes tha path information from source switch to dest switch.
 * This is a subroutine called by the Dijkstra_minimal_routing algorithm
 **/
void clear_path_collections(std::vector<flexfly_path *> &path_collection, 
								int dst_switch) {
	if (path_collection[dst_switch]) {
		//assert
		int path_len = path_collection[dst_switch]->path_length;
		for (int i = 0; i < path_len; i++) {
			if (path_collection[dst_switch]->path[i])
				delete path_collection[dst_switch]->path[i];
		}
		delete path_collection[dst_switch];
	}
};
