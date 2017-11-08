#include <vector>
#include <stack>
#include <queue>
#include <cassert>

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


