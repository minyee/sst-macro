#include <sstmac/hardware/router/routable.h>
#include <sstmac/hardware/pisces/pisces.h>
#include <queue>
#include <sprockit/factories/factory.h>
#include <sprockit/debug.h>
//#include <sprockit/serializable.h>
namespace sstmac{
namespace hw {

/** Contains information on the switch target
  *
 **/
struct switch_port_pair {
	int switch_id;
	int port;
};

class flexfly_packet : public pisces_default_packet {

//ImplementSerializable(flexfly_packet)

public:
	flexfly_packet() {
		uint32_t default_packet_size = 1024;// in bytes
		set_packet_data_size(default_packet_size);
		path_ = new std::queue<switch_port_pair*>(); 
	};
	
	~flexfly_packet() {
		if (path_)
			delete path_;
	}; 

	std::queue<switch_port_pair*>* route_path() {
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
		switch_port_pair* to_ret = path_->front();
		path_->pop();
		return to_ret;
	};

	void set_new_path(int dst_switch, int port_num) {
		switch_port_pair* spp = new switch_port_pair;
		spp->switch_id = dst_switch;
		spp->port = port_num;
		path_->push(spp);
		return;
	};

	uint32_t get_packet_data_size() const {
		return packet_data_size_;	
	}

	void set_packet_data_size(uint32_t size) {
		packet_data_size_ = size;
		return;
	};

	void set_pisces_packet() {

	}

	virtual int next_vc() const override {
		return 0;
	}
private: 
	std::queue<switch_port_pair*>* path_;
	uint32_t packet_data_size_;
	pisces_default_packet* pisces_packet_;
};


}
}
