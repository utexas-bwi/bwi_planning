#include <ros/ros.h>
#include <std_srvs/Empty.h>

#include <bwi_planning/cost_learner.h>
#include <bwi_planning/CostLearnerInterface.h>

boost::shared_ptr<bwi_planning::CostLearner> learner_;

bool incrementEpisode(std_srvs::Empty::Request &req, 
    std_srvs::Empty::Response &res) {
  learner_->finalizeEpisode();
}

bool addSample(bwi_planning::CostLearnerInterface::Request &req, 
    bwi_planning::CostLearnerInterface::Response &res) {
  res.status = "";
  res.success = learner_->addSample(req.location, req.door_from, req.door_to,
      req.cost);
  if (!res.success) {
    res.status = "Invalid arguments. Please recheck!";
  }
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "cost_learner");
  ros::NodeHandle n, private_nh("~");

  learner_.reset(new bwi_planning::CostLearner);

  ros::ServiceServer episode_service = 
    private_nh.advertiseService("increment_episode", incrementEpisode);
  ros::ServiceServer sample_service = 
    private_nh.advertiseService("add_sample", addSample);
  ros::spin();

  return 0;

}
