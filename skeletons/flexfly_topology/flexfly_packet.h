#include <routable.h>
#include <sstmac/hw/common/packet.h>
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
	std::queue< switch_port_pair >;
};

class flexfly_packet : public packet {
public:
	flexfly_packet();
	~flexfly_packet() {

	}
private: 
	path
};


}
}
