#include <boost/algorithm/string/join.hpp>
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <fstream>
#include <ros/ros.h>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

#include <bwi_planning/cost_learner.h>

#ifdef HAVE_NEW_YAMLCPP
namespace YAML {
  // The >> operator disappeared in yaml-cpp 0.5, so this function is
  // added to provide support for code written under the yaml-cpp 0.3 API.
  template<typename T>
  void operator >> (const YAML::Node& node, T& i)
  {
    i = node.as<T>();
  }
}
#endif

namespace bwi_planning {

  CostLearner::CostLearner () {

    ros::NodeHandle nh, private_nh("~");

    std::vector<std::string> unavailable_parameters;
    std::string door_file;
    if (!(private_nh.getParam("door_file", door_file))) {
      unavailable_parameters.push_back("door_file");
    }
    if (!(private_nh.getParam("values_file", values_file_))) {
      unavailable_parameters.push_back("values_file");
    }
    if (!(private_nh.getParam("lua_file", lua_file_))) {
      unavailable_parameters.push_back("lua_file");
    }
    private_nh.param("alpha", alpha_, 0.5);
    private_nh.param("use_exponential_weighting", 
        use_exponential_weighting_, true);

    if (unavailable_parameters.size() != 0) {
      std::string message = "Following neccessary params not available: " +
        boost::algorithm::join(unavailable_parameters, ", ");
      ROS_INFO_STREAM(message);
      throw std::runtime_error(message);
    }

    readDoorFile(door_file, doors_);
    prepareInputData();
  }

  void CostLearner::prepareInputData() {
    // Compute Iteration number
    iteration_ = 1;
    while (boost::filesystem::exists(values_file_ + 
          boost::lexical_cast<std::string>(iteration_))) {
      ++iteration_;
    }

    ROS_INFO_STREAM("Starting at iteration #" << iteration_);

    if (iteration_ == 1) {
      // Input data has no meaning. Initialize optimistically
      for (int idx = 0; idx < doors_.size(); ++idx) {
        for (int j = 0; j < doors_.size(); ++j) {
          if (j == idx) {
            continue;
          }
          if (doors_[j].approach_names[0] == doors_[idx].approach_names[0]) {
            std::string loc = doors_[j].approach_names[0];
            distance_estimates_[loc][idx][j] = 1.0f;
            distance_samples_[loc][idx][j] = 0;
          }
          if (doors_[j].approach_names[0] == doors_[idx].approach_names[1]) {
            std::string loc = doors_[j].approach_names[0];
            distance_estimates_[loc][idx][j] = 1.0f;
            distance_samples_[loc][idx][j] = 0;
          }
          if (doors_[j].approach_names[1] == doors_[idx].approach_names[0]) {
            std::string loc = doors_[j].approach_names[1];
            distance_estimates_[loc][idx][j] = 1.0f;
            distance_samples_[loc][idx][j] = 0;
          }
          if (doors_[j].approach_names[1] == doors_[idx].approach_names[1]) {
            std::string loc = doors_[j].approach_names[1];
            distance_estimates_[loc][idx][j] = 1.0f;
            distance_samples_[loc][idx][j] = 0;
          }
        }
      }
      // Write the lua file as it won't be available so that it can be used
      // in this turn
      writeValuesFile(0);
    } else {
      readValuesFile(iteration_ - 1);
    }
    writeLuaFile();
  }

  void CostLearner::writeLuaFile(std::string lua_file) {
    if (lua_file.empty()) {
      lua_file = lua_file_;
    }
    std::ofstream fout(lua_file.c_str());
    fout << "#begin_lua" << std::endl << std::endl;
    fout << "loc_table={}" << std::endl;
    int count = 0;
    BOOST_FOREACH(SIIFPair const& kv, distance_estimates_) {
      fout << "loc_table[\"" << kv.first << "\"] = " << count << std::endl;
      count++;
    }
    fout << "door_table={}" << std::endl;
    for (unsigned int i = 0; i < doors_.size(); ++i) {
      fout << "door_table[\"" << doors_[i].name << "\"] = " << i << std::endl;
    }
    fout << "function dis(a,b,c)" << std::endl;
    fout << "\td1 = door_table[tostring(a)]" << std::endl;
    fout << "\td2 = door_table[tostring(b)]" << std::endl;
    fout << "\tif d1==d2 then return 1 end" << std::endl;
    fout << "\tloc = loc_table[tostring(c)]" << std::endl;
    count = 0;
    BOOST_FOREACH(SIIFPair const& kv, distance_estimates_) {
      fout << "\tif loc==" << count << " then" << std::endl;
      BOOST_FOREACH(IIFPair const& kv2, kv.second) {
        fout << "\t\tif d1==" << kv2.first << " then" << std::endl;
        BOOST_FOREACH(IFPair const& kv3, kv2.second) {
          fout << "\t\t\tif d2==" << kv3.first << " then return " << 
            lrint(kv3.second) << " end" << std::endl;
        }
        fout << "\t\tend" << std::endl;
      }
      fout << "\tend" << std::endl;
      count++;
    }
    fout << "\treturn 1" << std::endl;
    fout << "end" << std::endl << std::endl;
    fout << "#end_lua." << std::endl;
    fout.close();
  }

  void CostLearner::writeValuesFile(int episode) {
    if (episode == -1) {
      episode = iteration_;
    }
    std::string out_file_name = values_file_ + 
      boost::lexical_cast<std::string>(episode);
    std::ofstream fout(out_file_name.c_str());
    BOOST_FOREACH(SIIIPair const& kv, distance_samples_) {
      fout << " - name: " << kv.first << std::endl;
      fout << "   costs: " << std::endl;
      BOOST_FOREACH(IIIPair const& kv2, kv.second) {
        BOOST_FOREACH(IIPair const& kv3, kv2.second) {
          fout << "     - from: " << kv2.first << std::endl;
          fout << "       to: " << kv3.first << std::endl;
          fout << "       cost: " <<
            distance_estimates_[kv.first][kv2.first][kv3.first] << std::endl;
          fout << "       samples: " << kv3.second << std::endl;
        }
      }
    }
    fout.close();
    std::string lua_file_name = values_file_ + "_distances" +
      boost::lexical_cast<std::string>(episode) + ".lua";
    writeLuaFile(lua_file_name);
  }

  void CostLearner::readValuesFile(int episode) {
    if (episode == -1) {
      episode = iteration_;
    }
    std::string in_file_name = values_file_ + 
      boost::lexical_cast<std::string>(episode);
    std::ifstream fin(in_file_name.c_str());

    YAML::Node doc;
#ifdef HAVE_NEW_YAMLCPP
    doc = YAML::Load(fin);
#else
    YAML::Parser parser(fin);
    parser.GetNextDocument(doc);
#endif

    for (size_t i = 0; i < doc.size(); ++i) {
      std::string loc;
      doc[i]["name"] >> loc;
      const YAML::Node &costs = doc[i]["costs"];
      for (size_t j = 0; j < costs.size(); ++j) {
        int from, to;
        costs[j]["from"] >> from;
        costs[j]["to"] >> to;
        costs[j]["cost"] >> distance_estimates_[loc][from][to];
        costs[j]["samples"] >> distance_samples_[loc][from][to];
      }
    }
  }

  bool CostLearner::addSample(const std::string& loc, int door_from, 
      int door_to, float cost) {

    if (door_from >= (int) doors_.size() || door_from < 0 ||
        door_to >= (int) doors_.size() || door_to < 0) {
      return false;
    }

    ROS_INFO_STREAM(std::string("Adding sample ") << 
        doors_[door_from].name << "->" <<
        doors_[door_to].name << ": " << cost);
    int samples = distance_samples_[loc][door_from][door_to];
    float old_cost = distance_estimates_[loc][door_from][door_to];
    float final_cost = 0;
    if (use_exponential_weighting_) {
      final_cost = (1.0f - alpha_) * old_cost + alpha_ * cost; 
    } else {
      if (samples != 0) {
        final_cost = (old_cost * samples + cost) / (samples + 1);
      } else {
        // First sample
        final_cost = cost;
      }
    }
    distance_estimates_[loc][door_from][door_to] = final_cost;
    distance_estimates_[loc][door_to][door_from] = final_cost;
    distance_samples_[loc][door_from][door_to] = samples + 1;
    distance_samples_[loc][door_to][door_from] = samples + 1;
  }

  bool CostLearner::addSample(const std::string& loc, 
      const std::string& door_from, const std::string& door_to, 
      float cost) {

    size_t door_from_idx = bwi_planning_common::resolveDoor(door_from, doors_);
    size_t door_to_idx = bwi_planning_common::resolveDoor(door_to, doors_);

    if (door_from_idx == bwi_planning_common::NO_DOOR_IDX || 
        door_to_idx == bwi_planning_common::NO_DOOR_IDX) {
      return false;
    }

    return addSample(loc, door_from_idx, door_to_idx, cost);
  }

  void CostLearner::finalizeEpisode() {
    writeLuaFile();
    writeValuesFile();
    ++iteration_;
    ROS_INFO_STREAM("Bumping cost learner to iteration #" << iteration_);
  }

} /* bwi_planning */
