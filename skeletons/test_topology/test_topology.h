#include <sstmac/hardware/topology/structured_topology.h>

namespace sstmac {
namespace hw {

class test_topology : public structured_topology {
  public:
  	FactoryRegister("test_topology", topology, test_topology, 
  		"A test topology to experiment if any topology should inherit from structured_topology instead");

  public:
  	test_topology(sprockit::sim_parameters* params);

  	~test_topology();

  	virtual int num_leaf_switches() const override {
  		return num_switches();
  	};

  	virtual int diameter() const override {
  		return 3; // assume diameter 3 topology
  	};

  	std::string to_string() const override {
  		return "xpress ring topology";
  	}
};

}
}