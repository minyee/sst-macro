#include <vector>
#include <stack>
#include <queue>

//use stack
void depth_first_search(std::vector< std::vector<int> >& connectivity_matrix, 
														int src_switch, 
														int dst_switch) {
	int curr_switch = src_switch;
	std::stack<int> switches_stack;
	switches_stack.push(src_switch);
	std::unordered_map<int, bool> visited_switches;
	bool found = false;
	std::map()
	while(!found && !switches_stack.empty()) {
		std::vector<int>& connectivity_vector = connectivity_matrix[curr_switch];
		for (int i = 0; i < connectivity_vector.size(); i++) {
			int target_switch = connectivity_vector[i];
			auto search = visited_switches.find(target_switch);
			
			if (search != visited_switches.end()) {

			} 
		}
		curr_switch = switches_stack.pop();
	}
	return;
}; 

// use queue
void breadth_first_search(std::vector< std::vector<int> >& connectivity_matrix,
														int src_switch, 
														int dst_switch) {

};