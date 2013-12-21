#include <ros/ros.h>
#include <std_srvs/Empty.h>

#include <bwi_planning/cost_learner.h>

boost::shared_ptr<bwi_planning::CostLearner> learner_;

bool incrementEpisode(std_srvs::Empty::Request &req, 
    std_srvs::Empty::Response &res) {
  learner_->finalizeEpisode();
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "cost_learner");
  ros::NodeHandle n;

  learner_.reset(new bwi_planning::CostLearner);

  ros::ServiceServer service = n.advertiseService("increment_episode", 
      incrementEpisode);
  ros::spin();

  return 0;

}
