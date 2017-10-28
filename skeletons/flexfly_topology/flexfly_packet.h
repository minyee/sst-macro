#include <routable.h>
#include <sstmac/hardware/pisces/pisces.h>

namespace sstmac{
namespace hw {

/** Contains information on the switch target
  *
 **/
struct switch_port_pair {
	int switch_id;
	int port;
};

struct flexfly_path {
	std::queue<switch_port_pair>;
};

class flexfly_packet : public pisces_payload {

ImplementSerializable(flexfly_packet);

public:
	flexfly_packet();
	
	~flexfly_packet() {
		if (path_)
			delete path_;
	}; 

	flexfly_path* route_path() {
		return path_;
	};

	/**
	 * pops the pointer to the next switch port pair. returns a nullptr if there are
	 * no more switches in the path. Note that this will decrement the count of the 
	 * switch__port_pair queue by 1.
	 **/
	switch_port_pair* next_switch() {
		if (!path_)
			return nullptr;
		switch_port_pair* to_ret = path_->first();
		path_->pop();
		return to_ret;
	}

	void set_new_path(int dst_switch, int port_num) {
		switch_port_pair* spp = new switch_port_pair;
		spp->switch_id = dst_switch;
		spp->port = port_num;
		path_->push(spp);
		return;
	}

private: 
	flexfly_path *path_;
};


}
}
