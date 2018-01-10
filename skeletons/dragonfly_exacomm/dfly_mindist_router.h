#include <sstmac/hardware/router/router.h>
#include "exacomm_dragonfly_topology.h"

#ifndef EXACOMM_DRAGONFLY_MINDIST_ROUTER
#define EXACOMM_DRAGONFLY_MINDIST_ROUTER

/**
 * @brief The dfly_mindist_router class
 * Router encapsulating the special routing computations that must occur on
 * a exacomm dragonfly topolog that may or may not be the canonical dragonfly topology.
 */
class dfly_mindist_router :
  public router
{
  FactoryRegister("dfly_mindist | ftree", router, dfly_mindist_router)
 public:
  virtual ~dfly_mindist_router();

  dfly_mindist_router(sprockit::sim_parameters* params, topology* top, network_switch* netsw);

  std::string to_string() const override {
    return "Min distance router class for the exacomm-defined Dragonfly topology router";
  }

 private:
  void route_to_switch(
    switch_id sw_addr,
    routable::path& path) override;

  /**
   * @brief choose_up_path
   * @return The selected path from the redundant (equivalent) set of minimal paths
   */
  int choose_up_minimal_path();

  /**
   * @brief number_paths
   * @param pkt The packet being routed by the fat-tree
   * @return The number of equivalent paths the packet can traverse
   *    on a minimal path to its destination switch.
   */
  int number_minimal_paths(packet* pkt) const;
}

#endif