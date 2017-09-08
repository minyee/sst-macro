#include <vector>

typedef size_t uint16_t;

class node {
public:
	node(int id) : id_(id) {

	}

	node(int id, size_t num_child) : id_(id), num_child_(num_child) {
		children_.reserve(num_child_);
	};

	~node() {};

	int num_child() const {
		return (int) num_child_;
	};

	void set_num_child(int num_child) {
		if (num_child > 0) {
			num_child_ = num_child_;
			children_.resize(num_child_);
		}
	};

	void set_child(int index, node* child_node) {
		if (index < num_child_ || index >= 0) {
			children_[index] = child_node;
		}
	};

	void get_child(int index) const {
		node* child = nullptr;
		if (index < num_child_ || index >= 0) {
			child = children_[index];
		}
		return child;
	};

	void get_id() const {
		return id_;
	};
private:
	size_t num_child_;
	
	int id_;
	
	std::vector<node*> children_;
};

