/**
 * Author: Min Yee Teh
 */

#include <sstmac/software/launch/job_launcher.h>
#include <sstmac/software/launch/launch_request_fwd.h>
#include <sstmac/software/launch/node_set.h>
#include <list>
namespace sstmac {
namespace sw{

/**
 * A greeedy job schduler essentiallu tries it's hardest to satisfy all the nodes
 * requested by a job. If it fails, it will just queue the job request first and then
 * try to satisfy it later.
 */
class greedy_job_launcher : public job_launcher {
	
 FactoryRegister("greedy", job_launcher, greedy_job_launcher)
 
 private:
  /**
   * @brief handle_new_launch_request As if a new job had been submitted with qsub or salloc.
   * The job_launcher receives a new request to launch an application, at which point
   * it can choose to launch the application immediately if node allocation succeeds.
   * @param request  An object specifying all the details (indexing, allocation, application type)
   *                of the application being launched
   * @param allocation [INOUT] The set of nodes allocated (but not yet indexed) for running
   *                           an application. This must fill out the allocation.
   * @return Whether the allocation succeeded. True means job can launch immediately.
   *         False means job must be delayed until another job finishes.
   */
  	virtual bool handle_launch_request(app_launch_request* request,
                                     ordered_node_set& allocation) override final;

    std::list<app_launch_request*> active_jobs_;
    std::list<app_launch_request*> pending_jobs_;
    int active_jobs_num_; 
 protected:
   /**
   * @brief stop_event_received Perform all necessary operations upon completion
   *            of a job. This indicates that new resources are available
   *            for running other jobs that might be queued
   * @param ev  An event describing the job that has finished
   */
  	virtual void stop_event_received(job_stop_event* ev) override final;
 public:	
  int active_jobs () const {return active_jobs_num_;};
 	 	/*
 	 * Class constructor
 	 */
 	greedy_job_launcher(sprockit::sim_parameters* params, operating_system* os) : job_launcher(params, os) {active_jobs_num_ = 0;}; //active_jobs_ = new std::list<app_launch_request*>()};
 	
 	/*
 	 * Class destructor
 	 */
 	~greedy_job_launcher() {}; //delete active_jobs_};
};


}
}