
/*
 *  This file is part of SST/macroscale:
 *               The macroscale architecture simulator from the SST suite.
 *  Copyright (c) 2009 Sandia Corporation.
 *  This software is distributed under the BSD License.
 *  Under the terms of Contract DE-AC04-94AL85000 with Sandia Corporation,
 *  the U.S. Government retains certain rights in this software.
 *  For more information, see the LICENSE file in the top
 *  SST/macroscale directory.
 */

#include <sstmac/hardware/switch/network_switch.h>
//#include <sstmac/hardware/logp/my_logp_switch.h>
#include "my_logp_switch.h"
#include <sstmac/hardware/nic/nic.h>
#include <sstmac/hardware/switch/dist_dummyswitch.h>
#include <sstmac/hardware/topology/topology.h>
#include <sstmac/hardware/interconnect/interconnect.h>
#include <sstmac/common/event_manager.h>
#include <sprockit/util.h>
#include <sprockit/sim_parameters.h>
#include <sstmac/software/launch/launch_event.h>

namespace sstmac {
namespace hw {

my_logp_switch::my_logp_switch(sprockit::sim_parameters *params, uint64_t id, event_manager *mgr) :
  network_switch(params, id, mgr, device_id::logp_overlay)
{
  sprockit::sim_parameters* link_params = params->get_namespace("link");
  sprockit::sim_parameters* ej_params = params->get_namespace("ejection");
  int my_id = params->get_int_param("id");
  std::cout << "YEAY HAHAHAHAH MY_LOG_P switch constructor with id: " << std::to_string(my_id) << std::endl;
  double net_bw = link_params->get_bandwidth_param("bandwidth");
  inverse_bw_ = 1.0/net_bw;
  if (link_params->has_param("send_latency")){
    hop_latency_ = link_params->get_time_param("send_latency");
  } else {
    hop_latency_ = link_params->get_time_param("latency");
  }

  double inj_bw = ej_params->get_optional_bandwidth_param("bandwidth", net_bw);
  inj_bw_inverse_ = 1.0/inj_bw;
  if (ej_params->has_param("send_latency")){
    inj_lat_ = ej_params->get_time_param("send_latency");
  } else {
    inj_lat_ = ej_params->get_time_param("latency");
  }
  dbl_inj_lat_ = 2*inj_lat_;

  inv_min_bw_ = std::max(inverse_bw_, inj_bw_inverse_);

  interconn_ = interconnect::static_interconnect(params, mgr);

  nics_.resize(top_->num_nodes());
  neighbors_.reserve(1000); //nproc - just reserve a large block for now

  init_links(params);
#if !SSTMAC_INTEGRATED_SST_CORE
  mtl_handler_ = new_handler(this, &my_logp_switch::handle);
#endif
}

my_logp_switch::~my_logp_switch()
{
#if !SSTMAC_INTEGRATED_SST_CORE
  delete mtl_handler_;
#endif
}

link_handler*
my_logp_switch::payload_handler(int port) const
{
#if SSTMAC_INTEGRATED_SST_CORE
  return new SST::Event::Handler<my_logp_switch>(
        const_cast<my_logp_switch*>(this), &my_logp_switch::handle);
#else
  return mtl_handler_;
#endif
}

void
my_logp_switch::connect_output(sprockit::sim_parameters *params,
                              int src_outport, int dst_inport,
                              event_handler *mod)
{
  //std::cout << "LogPSwitch->Connect Output: Src_outport: " << std::to_string(src_outport) << " Dst_inport: " << std::to_string(dst_inport) <<std::endl; 
  if (dst_inport == Node){
    node_id nid = src_outport;
    switch_debug("Connecting LogP to NIC %d", nid);
    nics_[nid] = mod;
  } else if (dst_inport == Switch){
    switch_id sid = src_outport;
    switch_debug("Connecting to LogP switch %d", sid);
    if (sid >= neighbors_.size()){
      neighbors_.resize(sid+1);
    }
    neighbors_[sid] = mod;
  } else {
    spkt_abort_printf("Invalid inport %d in my_logp_switch::connect_output", dst_inport);
  }
}

void
my_logp_switch::connect_input(sprockit::sim_parameters *params,
                              int src_outport, int dst_inport,
                              event_handler *mod)
{
  //no-op
}

void
my_logp_switch::incoming_message(message* msg, node_id src, node_id dst)
{
  bool local_src = nics_[src];
  timestamp delay(inv_min_bw_ * msg->byte_length()); //bw term
  int num_hops = 0;
  if (local_src){ //need to accumulate all the delay here
    //local staying local
    num_hops = top_->num_hops_to_node(src, dst);
    delay += num_hops * hop_latency_ + dbl_inj_lat_; //factor of 2 for in-out
  }
  switch_debug("incoming message over %d hops with extra delay %12.8e and inj lat %12.8e: %s",
               num_hops, delay.sec(), dbl_inj_lat_.sec(), msg->to_string().c_str());
  send_delayed_to_link(delay, nics_[dst], msg);
}

void
my_logp_switch::outgoing_message(message* msg, node_id src, node_id dst)
{
  //local going remote - just accumulate latency delay
  int num_hops = top_->num_hops_to_node(src, dst);
  timestamp delay = num_hops * hop_latency_; //factor of 2 for in-out

  int dst_switch = interconn_->node_to_logp_switch(dst);
  switch_debug("outgoing message over %d hops with extra delay %12.8e and inj lat %12.8e: %s",
               num_hops, delay.sec(), dbl_inj_lat_.sec(), msg->to_string().c_str());
  send_delayed_to_link(delay, dbl_inj_lat_, neighbors_[dst_switch], msg);
}

void
my_logp_switch::bcast_local_message(message* msg, node_id src)
{
  sw::start_app_event* lev = safe_cast(sw::start_app_event, msg);
  sw::task_mapping::ptr mapping = lev->mapping();
  int num_ranks = mapping->num_ranks();
  for (int i=0; i < num_ranks; ++i){
    node_id dst_node = mapping->rank_to_node(i);
    auto dst_nic = nics_[dst_node];
    bool local_dst = dst_nic;
    bool local_src = nics_[src];

    if (local_dst){
      sw::start_app_event* new_lev = lev->clone(i, src, dst_node);
      int num_hops = top_->num_hops_to_node(src, dst_node);
      timestamp delay = num_hops * hop_latency_;
      if (local_src){ //have to accumulate inj latency here
        delay += dbl_inj_lat_; //factor of 2 for in-out
      }
      send_delayed_to_link(delay, dst_nic, new_lev);
    }
  }
  //this one not needed anymore
  delete lev;
  std::cout << "going out of bcast_local_message" << std::endl;
}

void
my_logp_switch::forward_bcast_message(message* msg, node_id dst)
{
  int dst_switch = interconn_->node_to_logp_switch(dst);
  //only accumulate inj lat - exact hop latency gets added on the other side
  send_delayed_to_link(timestamp(), dbl_inj_lat_, neighbors_[dst_switch], msg);
}

void
my_logp_switch::handle(event* ev)
{
  //this should only handle messages
  std::cout << "handle more than once is just pretty bad" << std::endl;
  message* msg = safe_cast(message, ev);
  node_id dst = msg->toaddr();
  node_id src = msg->fromaddr();
  bool local_dst = nics_[dst];

  switch_debug("handling message %d->%d of size %ld: %s",
               src, dst, msg->byte_length(),
               msg->to_string().c_str());

  if (msg->is_bcast()){
    if (local_dst){
      bcast_local_message(msg, src);
    } else {
      forward_bcast_message(msg, dst);
    }
  } else if (local_dst){
    incoming_message(msg, src, dst);
  } else { //locall going remote
    outgoing_message(msg, src, dst);
  }
}



}
}