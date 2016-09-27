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

#include <sstmac/common/event_scheduler.h>
#include <sstmac/common/sstmac_config.h>
#include <sstmac/common/event_handler.h>
#include <sstmac/common/event_manager.h>
#include <sstmac/common/sst_event.h>
#include <sstmac/common/sstmac_env.h>
#include <sstmac/hardware/node/node.h>
#include <sprockit/sim_parameters.h>

#if SSTMAC_INTEGRATED_SST_CORE
#include <sstmac/sst_core/connectable_wrapper.h>
#endif

#define test_schedule(x) \
  if (dynamic_cast<integrated_connectable_wrapper*>(x)) abort()

namespace sstmac {


void
event_scheduler::cancel_all_messages()
{
#if SSTMAC_INTEGRATED_SST_CORE
  spkt_throw(sprockit::unimplemented_error,
    "event_scheduler::cancel_all_messages: cannot cancel messages currently in integrated core");
#else
  eventman_->cancel_all_messages(event_location());
#endif
}

void
event_scheduler::init(unsigned int phase)
{
#if SSTMAC_INTEGRATED_SST_CORE
  SSTIntegratedComponent::init(phase);
#endif
}

void
event_scheduler::setup()
{
#if SSTMAC_INTEGRATED_SST_CORE
  SSTIntegratedComponent::setup();
#endif
}

#if SSTMAC_INTEGRATED_SST_CORE

void
event_scheduler::schedule_now(event_handler* handler, event* ev)
{
  //this better be a zero latency link
  schedule(SST::SimTime_t(0), handler, ev);
}

void
event_scheduler::schedule_now(event_queue_entry* ev)
{
  self_link_->send(ev);
}

void
event_scheduler::send_self_event(timestamp arrival, event* ev)
{
  SST::SimTime_t delay = extra_delay(arrival);
  self_link_->send(delay, time_converter_, ev);
}

void
event_scheduler::send_delayed_self_event(timestamp delay, event* ev)
{
  SST::SimTime_t sst_delay = delay.ticks_int64();
  self_link_->send(sst_delay, time_converter_, ev);
}

void
event_scheduler::send_now_self_event(event* ev)
{
  self_link_->send(ev);
}

void
event_scheduler::send_self_event_queue(timestamp arrival, event_queue_entry* ev)
{
  SST::SimTime_t delay = extra_delay(arrival);
  self_link_->send(delay, time_converter_, ev);
}

void
event_scheduler::send_delayed_self_event_queue(timestamp delay, event_queue_entry* ev)
{
  SST::SimTime_t sst_delay = delay.ticks_int64();
  self_link_->send(sst_delay, time_converter_, ev);
}

void
event_scheduler::send_now_self_event_queue(event_queue_entry* ev)
{
  self_link_->send(ev);
}

void
event_scheduler::schedule(SST::SimTime_t delay, event_handler* handler, event* ev)
{
  event_queue_entry* evq = new handler_event_queue_entry(ev, handler, event_location());
  if (handler->link()) abort();
  self_link_->send(delay, time_converter_, evq);
}

void
event_scheduler::send_to_link(event_handler* handler, event *ev)
{
  SST::Link* link = handler->link();
  if (link){
    //we ignore the latency here
    link->send(0, time_converter_, ev);
  } else {
    //oh - there is no link, you lied to me
    self_link_->send(ev);
  }
}

void
event_scheduler::send_to_link(timestamp enter, timestamp lat,
                              event_handler* handler, event *ev)
{
  SST::Link* link = handler->link();
  if (link){
    //we ignore the latency here
    link->send(extra_delay(enter), time_converter_, ev);
  } else {
    //oh - there is no link, you lied to me
    schedule(enter + lat, handler, ev);
  }
}

void
event_scheduler::send_delayed_to_link(timestamp extra_delay, timestamp lat,
                              event_handler* handler, event *ev)
{
  SST::Link* link = handler->link();
  if (link){
    //we ignore the latency here
    link->send(SST::SimTime_t(extra_delay.ticks_int64()), time_converter_, ev);
  } else {
    //oh - there is no link, you lied to me
    schedule_delay(extra_delay + lat, handler, ev);
  }
}

void
event_scheduler::send_delayed_to_link(timestamp extra_delay,
                              event_handler* handler, event *ev)
{
  SST::Link* link = handler->link();
  if (link){
    //we ignore the latency here
    link->send(SST::SimTime_t(extra_delay.ticks_int64()), time_converter_, ev);
  } else {
    //oh - there is no link, you lied to me
    schedule_delay(extra_delay, handler, ev);
  }
}

void
event_scheduler::schedule(timestamp t,
                          event_handler* handler,
                          event* ev)
{
  schedule(extra_delay(t), handler, ev);
}

void
event_scheduler::schedule(timestamp t, event_queue_entry *ev)
{
  send_self_event_queue(t, ev);
}

void
event_scheduler::schedule_delay(
  timestamp delay,
  event_handler* handler,
  event* ev)
{
  schedule(SST::SimTime_t(delay.ticks_int64()), handler, ev);
}

void
event_scheduler::schedule_delay(timestamp delay, event_queue_entry *ev)
{
  self_link_->send(SST::SimTime_t(delay.ticks_int64()), time_converter_, ev);
}

#else
void
event_scheduler::send_to_link(timestamp enter, timestamp lat,
                              event_handler* handler, event *ev)
{
  timestamp arrival = enter + lat;
  schedule(arrival, handler, ev);
}

void
event_scheduler::send_to_link(event_handler *lnk, event *ev)
{
  schedule_now(lnk, ev);
}

void
event_scheduler::send_delayed_to_link(timestamp extra_delay, timestamp lat,
                              event_handler* handler, event *ev)
{
  timestamp arrival = now() + extra_delay + lat;
  schedule(arrival, handler, ev);
}

void
event_scheduler::send_delayed_to_link(timestamp extra_delay,
                              event_handler* handler, event *ev)
{
  timestamp arrival = now() + extra_delay;
  schedule(arrival, handler, ev);
}

void
event_scheduler::schedule_now(event_handler *handler, event* ev)
{
  event_queue_entry* qev = new handler_event_queue_entry(ev, handler, event_location());
  schedule(now(), qev);
}

void
event_scheduler::send_self_event(timestamp arrival, event* ev)
{
  schedule(arrival, self_handler_, ev);
}

void
event_scheduler::send_delayed_self_event(timestamp delay, event* ev)
{
  schedule_delay(delay, self_handler_, ev);
}

void
event_scheduler::send_now_self_event(event* ev)
{
  schedule_now(self_handler_, ev);
}

void
event_scheduler::send_self_event_queue(timestamp arrival, event_queue_entry *ev)
{
  schedule(arrival, ev);
}

void
event_scheduler::send_delayed_self_event_queue(timestamp delay, event_queue_entry* ev)
{
  schedule_delay(delay, ev);
}

void
event_scheduler::send_now_self_event_queue(event_queue_entry* ev)
{
  schedule_now(ev);
}

void
event_scheduler::schedule_delay(
  timestamp delay,
  event_handler* handler,
  event* ev)
{
  schedule(now() + delay, handler, ev);
}

void
event_scheduler::schedule_now(event_queue_entry* ev)
{
  schedule(now(), ev);
}

void
event_scheduler::schedule_delay(timestamp delay, event_queue_entry* ev)
{
  schedule(now() + delay, ev);
}

void
event_scheduler::schedule(timestamp t, event_queue_entry* ev)
{
#if SSTMAC_SANITY_CHECK
  double delta_t = (t - now()).sec();
  if (delta_t < -1e-9) {
    spkt_throw_printf(sprockit::illformed_error,
                     "time has gone backwards %8.4e seconds", delta_t);
  }
#endif
  eventman_->schedule(t, seqnum_++, ev);
}

void
event_scheduler::register_stat(stat_collector *coll)
{
  eventman_->register_stat(coll);
}

void
event_scheduler::ipc_schedule(timestamp t, event_handler* handler, event* ev)
{
  eventman_->ipc_schedule(t, handler->event_location(), event_location(), seqnum_++, ev);
}

void
event_scheduler::sanity_check(timestamp t)
{
  if (eventman_ == 0) {
    spkt_throw_printf(sprockit::null_error,
                     "event_scheduler::schedule: null event manager");
  }
  double delta_t = (t - now()).sec();
  if (delta_t < -1e-9) {
    spkt_throw_printf(sprockit::illformed_error,
      "time has gone backwards %8.4e seconds",
      delta_t);
  }
}

void
event_scheduler::multithread_schedule(int src_thread, int dst_thread,
  timestamp t, event_queue_entry* ev)
{
  debug_printf(sprockit::dbg::event_manager,
      "On %s, scheduling event at t=%12.8e srcthread=%d dstthread=%d",
      sprockit::to_string(this).c_str(), t.sec(), src_thread, dst_thread);
  if (dst_thread != event_handler::null_threadid
     && dst_thread != src_thread){
    ev->set_time(t);
    eventman_->multithread_schedule(
      src_thread, dst_thread,
      seqnum_++, ev);
  } else {
    eventman_->schedule(t, seqnum_++, ev);
  }
}

void
event_scheduler::schedule(timestamp t,
                          event_handler* handler,
                          event* ev)
{
#if SSTMAC_SANITY_CHECK
  sanity_check(t);
#endif
  if (handler->ipc_handler()){
    ipc_schedule(t, handler, ev);
  }
  else {
    event_queue_entry* qev = new handler_event_queue_entry(ev, handler, event_location());
#if SSTMAC_USE_MULTITHREAD
    multithread_schedule(thread_id(), handler->thread_id(), t, qev);
#else
    eventman_->schedule(t, seqnum_++, qev);
#endif
  }
}
#endif

void
event_subscheduler::init(unsigned int phase)
{
  //do nothing
}

void
event_subscheduler::setup()
{
  //do nothing
}

void
event_subscheduler::schedule(timestamp t,
                          event_handler* handler,
                          event* ev)
{
  parent_->schedule(t, handler, ev);
}

void
event_subscheduler::schedule_delay(timestamp t, event_queue_entry *ev)
{
  parent_->schedule_delay(t, ev);
}

void
event_subscheduler::schedule(timestamp t, event_queue_entry *ev)
{
  parent_->schedule(t, ev);
}

void
event_subscheduler::schedule_now(event_handler *handler, event* ev)
{
  parent_->schedule_now(handler, ev);
}

void
event_subscheduler::send_self_event(timestamp arrival, event* ev)
{
  parent_->schedule(arrival, self_handler_, ev);
}

void
event_subscheduler::send_delayed_self_event(timestamp delay, event* ev)
{
  parent_->schedule_delay(delay, self_handler_, ev);
}

void
event_subscheduler::send_now_self_event(event* ev)
{
  parent_->schedule_now(self_handler_, ev);
}

void
event_subscheduler::send_to_link(event_handler *lnk, event *ev)
{
  parent_->send_to_link(lnk, ev);
}

void
event_subscheduler::send_to_link(timestamp enter, timestamp lat, event_handler *lnk, event *ev)
{
  parent_->send_to_link(enter, lat, lnk, ev);
}

void
event_subscheduler::send_delayed_to_link(timestamp extra_delay, event_handler *link, event *ev)
{
  parent_->send_delayed_to_link(extra_delay, link, ev);
}

void
event_subscheduler::send_delayed_to_link(timestamp extra_delay, timestamp lat,
                                         event_handler *link, event *ev)
{
  parent_->send_delayed_to_link(extra_delay, lat, link, ev);
}

void
event_subscheduler::send_self_event_queue(timestamp arrival, event_queue_entry *ev)
{
  parent_->send_self_event_queue(arrival, ev);
}

void
event_subscheduler::send_delayed_self_event_queue(timestamp delay, event_queue_entry* ev)
{
  parent_->send_delayed_self_event_queue(delay, ev);
}

void
event_subscheduler::send_now_self_event_queue(event_queue_entry* ev)
{
  parent_->send_now_self_event_queue(ev);
}

void
event_subscheduler::schedule_delay(
  timestamp delay,
  event_handler* handler,
  event* ev)
{
  parent_->schedule_delay(delay, handler, ev);
}

void
event_subscheduler::schedule_now(event_queue_entry* ev)
{
  parent_->send_now_self_event_queue(ev);
}


} // end of namespace sstmac

