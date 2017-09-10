#include "data_structures.h"
#include <vector>

void node_configure(int** group_connectivity_matrix, size_t size);

void node_configure_recursive(node* parent, std::vector<int> child_id, int depth);

void generate_butterfly(int num_groups,  std::vector<node*>& groups, std::vector<node*>& optical_switches);

void link_butterfly(std::vector<node*>& groups, std::vector<node*>& optical_switches);

void form_butterfly(int num_groups, std::vector<node*>& groups, std::vector<node*>& optical_switches);

void delete_butterfly();

void canonical_dragonfly_config_greedy(int num_groups, std::vector<std::vector<int>>& optical_inout_connections);

node* dfs(node* curr_node, int depth, int target_group_id);
