#include <sstmac/software/launch/greedy_job_launcher.h>
#include <sstmac/software/launch/launch_request_fwd.h>
#include <sstmac/software/launch/launch_request.h>
#include <sstmac/software/launch/node_set.h>
#include <sstmac/software/process/operating_system.h>
#include <sstmac/software/launch/launch_event.h>
#include <sstmac/hardware/topology/topology.h>
#include <iostream>
#include <list>


namespace sstmac {
namespace sw {
bool greedy_job_launcher::handle_launch_request(app_launch_request* request,
                                     ordered_node_set& allocation) {
	int required_nodes = request->nproc() / request->procs_per_node();
	required_nodes = request->nproc() % request->procs_per_node() ? required_nodes + 1 : required_nodes;
	std::cout << "WE ARE IN GREEDY JOB LAUNCHER: HANDLE LAUNCH REQUEST" << std::endl;
	if (required_nodes > topology_->num_nodes())
		spkt_abort_printf("more nodes requested that what the hardware of the system can offer");
	bool success = request->request_allocation(available_, allocation);
	//std::cout << "AM I CALLED AT ALL?" << std::endl;
	if (!success) {
		std::cout << "Should see this 3 times?" << std::endl;
		pending_jobs_.push_back(request);
		if (pending_jobs_.empty()) {
			std::cout << "SHOULD NOT SEE THIS AT ALL" << std::endl;
		}
		return false;
	}

	for (const node_id& nid : allocation){
	    if (available_.find(nid) == available_.end()){
	      pending_jobs_.push_back(request);
	      return false;
	    }
	}
	// implementing the double pass for both more memory and runtime friendly
	for (const node_id& nid : allocation) {
	  	available_.erase(nid);
	}
	active_jobs_.push_back(request);
	active_jobs_num_++;
	return true;
};


void greedy_job_launcher::stop_event_received(job_stop_event* ev) {
	
	//launch_event* le = dynamic_cast<launch_event*>(ev);
	app_id app = 0;
	if (!ev) {
		app = ev->aid();
		for (app_launch_request* request : active_jobs_) {
			if (request->aid() == app) {
				active_jobs_.remove(request);
				active_jobs_num_--;
				break;
			} 
		}
		os_->decrement_app_refcount();
		
	}
	if (!pending_jobs_.empty()) {
			std::cout << "EVER GOT HERE AT ALL?" << std::endl;
			app_launch_request* request = pending_jobs_.front();
			pending_jobs_.pop_front();
			active_jobs_.push_back(request);
			active_jobs_num_++;
			job_launcher::incoming_launch_request(request);
		}
	// try to schedule for another app to run

	if (active_jobs_num_ != active_jobs_.size()) {
		spkt_abort_printf("WE FUCKED UP");
	}

	// have to somehow traslate the job_stop_event to the actual job request
	
	
};
}
}