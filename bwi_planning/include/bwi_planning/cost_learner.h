#ifndef BWI_PLANNING_COST_LEARNER_H
#define BWI_PLANNING_COST_LEARNER_H

#include <bwi_planning_common/structures.h>

namespace bwi_planning {

  typedef std::pair<const int, float> IFPair;
  typedef std::pair<const int, int> IIPair;
  typedef std::pair<const int, std::map<int, float> > IIFPair;
  typedef std::pair<const int, std::map<int, int> > IIIPair;
  typedef std::pair<const std::string, std::map<int, std::map< int, float> > > 
    SIIFPair;
  typedef std::pair<const std::string, std::map<int, std::map< int, int> > > 
    SIIIPair;

  class CostLearner {

    public:

      CostLearner ();

      void prepareInputData();
      void writeLuaFile(std::string lua_file = "");
      void writeValuesFile(int episode = -1);
      void readValuesFile(int episode = -1);

      void addSample(std::string loc, int door_from, int door_to, float cost);
      void finalizeEpisode();

    private:

      std::vector<bwi_planning_common::Door> doors_;
      std::map<std::string, std::map<int, std::map<int, float> > > 
        distance_estimates_;
      std::map<std::string, std::map<int, std::map<int, int> > > 
        distance_samples_;

      std::string values_file_;
      std::string lua_file_;

      double alpha_;
      bool use_exponential_weighting_;
      int iteration_;
  };
}

#endif /* end of include guard: BWI_PLANNING_COST_LEARNER_H */
