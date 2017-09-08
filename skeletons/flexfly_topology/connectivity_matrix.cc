#include ""
#include "data_structures.h"

void node_configure(int** group_connectivity_matrix, size_t size) {
	
}

void node_configure_recursive(node* parent, std::vector<int> child_id, int depth) {
	// base case
	if (depth == 0 || parent == nullptr) {
		return;
	} 
	for (int i = 0; i < child_id.size(); i++) {
		node* child = new node(child_id[i]);
	}

}

void generate_butterfly(int num_groups, ) {
	//std::vector<node*>& groups = new std::vector<node*>[num_groups];
	int num_optical_switches = num_groups - 1;
	//std::vector<node*>& optical_switches = new std::vector<node*>[num_optical_switches];
	for (int i = 0; i < num_groups; i++) {
		groups[i] = new node(i);
		if (i < num_groups - 1) 
			optical_switches = new node(i);
	}
}

void link_butterfly(std::vector<node*>& groups, std::vector<node*>& optical_switches) {
	for (int i = 0; i < groups.size(); i++) {
		node* curr_group_node = groups[i];
		for (int j = 0; j < optical_switches.size(); j++) {
			node* curr_optical_switch = optical_switches[j];
			curr_group_node->set_child(j, curr_optical_switch);
			curr_optical_switch->set_child(i, curr_group_node);
		}
	}
}

void form_butterfly(int num_groups, std::vector<node*>& groups, std::vector<node*>& optical_switches) {
	generate_butterfly(num_groups, groups, optical_switches);
	link_butterfly(groups,optical_switches);
}

void delete_butterfly() {

}

void canonical_dragonfly_config_greedy(int num_groups, std::vector<std::vector<int>>& optical_inout_connections) {
	std::vector<node*> groups = std::vector(num_groups);
	std::vector<node*> optical_switches = std::vector(num_groups - 1);
	form_butterfly(num_groups, groups, optical_switches);
	// for all switches, find the optical switch that gets them to every other switch
	for (int i = 0; i < num_groups; i++) {
		node* curr_group = groups[i];
		int curr_group_id = curr_group->get_id();
		for (int j = 0; j < num_groups; j++) {
			if (i == j) 
				continue;
			// NOTE: j is the target group's id
			node* optical_switch = dfs(curr_node->get_child(j), 2, j);
			if (optical_switch == nullptr) {
				std::cout << "FAILSSSSS" << std::endl;
			}
			optical_switch->set_child(j, nullptr); // break the links so they no longer exist in the graph
			curr_group->set_child(optical_switch->get_id(), nullptr);
			// TODO: Now here need to set the configuration
			// CHECK THIS PART (START)
			std::vector<int>& inout_connection_for_optical_switch = optical_inout_connections[optical_switch->get_id()];
			inout_connection_for_optical_switch[i] = j; 
			// CHECK THIS PART (END)
		}
	}
}

/**
 * short for depth-first search
 */ 
node* dfs(node* curr_node, int depth, int target_group_id) {
	// last iteration
	if (depth == 1) {
		node* found_node = target_group_id == curr_node->get_id() ? curr_node : nullptr;
		return found_node;
	} else {
		for (int i = 0; i < curr_node->num_child(); i++) {
			node* found_node = dfs(curr_node->get_child(i), depth - 1, target_group_id);
			if (found_node != nullptr) {
				// return curr node so that the group will know which 
				// optical switch can be connected to the destination group
				found_node = curr_node; 
			}
			return found_node;
		}
		dfs()
	}
}