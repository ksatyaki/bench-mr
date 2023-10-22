//
// Created by ksatyaki on 08/10/23.
//

#include <base/CarStateSpace.h>
#include <base/environments/MRPTGridMap.h>
#include <ompl/base/samplers/UniformValidStateSampler.h>
#include <ompl/base/samplers/informed/PathLengthDirectInfSampler.h>
#include <ompl/mod/objectives/DTCOptimizationObjective.h>
#include <ompl/mod/objectives/IntensityMapOptimizationObjective.h>
#include <ompl/mod/objectives/UpstreamCriterionOptimizationObjective.h>
#include <ompl/mod/samplers/DijkstraSampler.h>
#include <ompl/mod/samplers/HybridSampler.h>
#include <ompl/mod/samplers/IntensityMapSampler.h>

#include <boost/program_options.hpp>

namespace og = ompl::geometric;
namespace ob = ompl::base;
namespace om = ompl::MoD;

int main(int argn, char* args[]) {
  // Add boost program options for program
  namespace po = boost::program_options;
  po::options_description desc("Options");
  desc.add_options()("help", "Print help message");
  desc.add_options()("yaml", po::value<std::string>(), "Yaml filename");
  desc.add_options()("objective", po::value<std::string>(), "Objective type");
  desc.add_options()("cliffmap", po::value<std::string>(), "Cliffmap filename");
  desc.add_options()("gmmtmap", po::value<std::string>(), "GMMTMap filename");
  desc.add_options()("intensitymap", po::value<std::string>(), "IntensityMap filename");
  desc.add_options()("sampler", po::value<std::string>(), "Sampler type");
  desc.add_options()("sampling_bias", po::value<double>(), "Sampling bias");
  desc.add_options()("dijkstra_cell_size", po::value<double>(), "Dijkstra cell size");
  desc.add_options()("file", po::value<std::string>(), "Output file");

  po::variables_map vm;
  po::store(po::parse_command_line(argn, args, desc), vm);
  po::notify(vm);

  if (vm.count("help")) {
    std::cout << desc << "\n";
    return 1;
  }

  // Open file to write even if it doesn't exist.
  FILE* file = fopen((vm["file"].as<std::string>() + std::string("-all.txt")).c_str(), "w");
  FILE* filevalid = fopen((vm["file"].as<std::string>() + std::string("-valid.txt")).c_str(), "w");

  auto objective = vm["objective"].as<std::string>();
  auto cliffmap_filename = vm["cliffmap"].as<std::string>();
  auto gmmtmap_filename = vm["gmmtmap"].as<std::string>();
  auto intensitymap_filename = vm["intensitymap"].as<std::string>();
  auto sampler_type = vm["sampler"].as<std::string>();
  auto sampling_bias = vm["sampling_bias"].as<double>();
  auto dijkstra_cell_size = vm["dijkstra_cell_size"].as<double>();

  auto map = std::make_shared<MRPTGridMap>(vm["yaml"].as<std::string>());

  // Create a car state space
  auto car_state_space = std::make_shared<ob::CarStateSpace>(0.5);
  car_state_space->setBounds(map->getBounds());
  auto si = std::make_shared<ob::SpaceInformation>(car_state_space);

  std::shared_ptr<ompl::base::OptimizationObjective> opt_obj_;
  if (objective == "cliff-euc") {
    opt_obj_ = std::make_shared<ompl::MoD::UpstreamCriterionOptimizationObjective>(
        si, ompl::MoD::MapType::CLiFFMap, cliffmap_filename, 1, 1, 0.1, sampler_type, intensitymap_filename,
        sampling_bias, false);
    std::dynamic_pointer_cast<om::MoDOptimizationObjective>(opt_obj_)->setDijkstraCellSize(dijkstra_cell_size);
  } else if (objective == "gmmt-euc") {
    opt_obj_ = std::make_shared<ompl::MoD::UpstreamCriterionOptimizationObjective>(
        si, ompl::MoD::MapType::GMMTMap, gmmtmap_filename, 1, 1, 0.1, sampler_type, intensitymap_filename,
        sampling_bias, false);
    std::dynamic_pointer_cast<om::MoDOptimizationObjective>(opt_obj_)->setDijkstraCellSize(dijkstra_cell_size);
  } else if (objective == "cliff-dtc") {
    opt_obj_ =
        std::make_shared<ompl::MoD::DTCOptimizationObjective>(si, cliffmap_filename, intensitymap_filename, 1.0, 1.0,
                                                              0.02, 1.0, 10, true, sampler_type, sampling_bias, false);
    std::dynamic_pointer_cast<om::MoDOptimizationObjective>(opt_obj_)->setDijkstraCellSize(dijkstra_cell_size);
  } else if (objective == "intensity") {
    opt_obj_ = std::make_shared<ompl::MoD::IntensityMapOptimizationObjective>(si, intensitymap_filename, 1, 1, 0.2,
                                                                              sampler_type, sampling_bias, false);
    std::dynamic_pointer_cast<om::MoDOptimizationObjective>(opt_obj_)->setDijkstraCellSize(dijkstra_cell_size);
  } else {
    OMPL_INFORM("Invalid objective type");
    exit(1);
  }

  auto problem_def = std::make_shared<ob::ProblemDefinition>(si);
  problem_def->setOptimizationObjective(opt_obj_);

  // goal: [47.690, -18.848, 0.84]
  // start: [-19.575, 12.390, -0.84]
  // Start and goal states
  ob::ScopedState<ob::CarStateSpace> start(car_state_space);
  start->setXY(47.690, -18.848);
  start->setYaw(0.84);
  ob::ScopedState<ob::CarStateSpace> goal(car_state_space);
  goal->setXY(-19.575, 12.390);
  goal->setYaw(0.84);
  problem_def->setStartAndGoalStates(start, goal);

  // Set the state validity checker to the MRPT thing.
  si->setStateValidityChecker([map](const ob::State* state) {
    return !map->collides(state->as<ob::CarStateSpace::StateType>()->getX(),
                          state->as<ob::CarStateSpace::StateType>()->getY());
  });

  // allocate a sampler for the problem definition
  auto sampler = opt_obj_->allocInformedStateSampler(problem_def, std::numeric_limits<unsigned int>::max());

  for (int i = 0; i < 1000; i++) {
    ob::State* state = si->allocState();
    sampler->sampleUniform(state, ompl::base::Cost(std::numeric_limits<double>::max()));
    // Print a csv style ouput of the sampling function, cost function and the state:
    fprintf(file, "%f,%f,%f,%s-%0.2f-%0.1f,%s\n", state->as<ob::CarStateSpace::StateType>()->getX(),
            state->as<ob::CarStateSpace::StateType>()->getY(), state->as<ob::CarStateSpace::StateType>()->getYaw(),
            sampler_type.c_str(), sampling_bias, dijkstra_cell_size, objective.c_str());

    // free the state
    si->freeState(state);
  }

  // a vector to collect valid samples
  std::vector<ob::State*> valid_states;

  // sample a several states until 1000 are valid
  for (int valid = 0; valid < 1000;) {
    ob::State* state = si->allocState();
    sampler->sampleUniform(state, ompl::base::Cost(std::numeric_limits<double>::max()));
    if (si->isValid(state)) {
      fprintf(filevalid, "%f,%f,%f,%s-%0.2f-%0.1f,%s\n", state->as<ob::CarStateSpace::StateType>()->getX(),
              state->as<ob::CarStateSpace::StateType>()->getY(), state->as<ob::CarStateSpace::StateType>()->getYaw(),
              sampler_type.c_str(), sampling_bias, dijkstra_cell_size, objective.c_str());
      valid_states.push_back(state);
      valid++;
    }
  }

  for (auto& state : valid_states) {
    si->freeState(state);
  }
  // close the file
  fclose(file);

  return 0;
}
