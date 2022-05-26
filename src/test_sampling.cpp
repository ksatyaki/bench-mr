//
// Created by ksatyaki on 25/05/22.
//

#include <ompl/mod/samplers/IntensityMapSampler.h>
#include <ompl/mod/samplers/DijkstraSampler.h>
#include <ompl/mod/objectives/UpstreamCriterionOptimizationObjective.h>
#include <ompl/mod/objectives/DTCOptimizationObjective.h>
#include <ompl/mod/objectives/IntensityMapOptimizationObjective.h>

#include "base/CarStateSpace.h"

#include <boost/program_options.hpp>


int main(int argn, char *args[]) {
    namespace po = boost::program_options;

    po::options_description desc("Allowed options");
    desc.add_options()("help", "produce this help message")(
            "cliffmap,c", po::value<std::string>()->required(), "cliff map file name")(
            "gmmtmap,g", po::value<std::string>()->required(), "gmmt map file name")(
            "intensitymap,i", po::value<std::string>()->required(), "intensity map file name")(
            "sampler,s", po::value<std::string>()->required(), "What sampler to use?");

    po::variables_map vm;
    po::store(po::parse_command_line(argn, args, desc), vm);
    po::notify(vm);

    ompl::base::StateSpacePtr space(new ompl::base::CarStateSpace(0.5));
    ompl::base::SpaceInformationPtr si(new ompl::base::SpaceInformation(space));

    ompl::base::OptimizationObjectivePtr obj_dtc = std::make_shared<ompl::MoD::DTCOptimizationObjective>(si,
                                                                                                         vm["cliffmap"].as<std::string>(),
                                                                                                         vm["intensitymap"].as<std::string>(),
                                                                                                         1.0, 1.0, 0.05,
                                                                                                         1.0, 10, true);
    ompl::base::OptimizationObjectivePtr obj_gmmt = std::make_shared<ompl::MoD::UpstreamCriterionOptimizationObjective>(
            si, ::MoD::GMMTMap(vm["gmmtmap"].as<std::string>()), 1.0, 1.0, 0.2);
    ompl::base::OptimizationObjectivePtr obj_cliff = std::make_shared<ompl::MoD::UpstreamCriterionOptimizationObjective>(
            si, ::MoD::CLiFFMap(vm["cliffmap"].as<std::string>()), vm["intensitymap"].as<std::string>(), 1.0, 1.0, 0.2);

    if (vm["sampler"].as<std::string>() == "gmmt") {
        si->clearValidStateSamplerAllocator();
        si->setValidStateSamplerAllocator(
                [=](auto &&PH1) {
                    return ompl::MoD::DijkstraSampler::allocate(
                            std::forward<decltype(PH1)>(PH1),
                            obj_gmmt,
                            {-5.0, -5.0,
                             EIGEN_PI / 4.0},
                            {19.0, 19.0, EIGEN_PI / 4.0},
                            0.25,
                            0.05);
                });
    } else if (vm["sampler"].as<std::string>() == "dtc") {
        si->clearValidStateSamplerAllocator();
        si->setValidStateSamplerAllocator(
                [=](auto &&PH1) {
                    return ompl::MoD::DijkstraSampler::allocate(
                            std::forward<decltype(PH1)>(PH1),
                            obj_dtc,
                            {-5.0, -5.0,
                             EIGEN_PI / 4.0},
                            {19.0, 19.0, EIGEN_PI / 4.0},
                            0.25,
                            0.05);
                });
    } else if (vm["sampler"].as<std::string>() == "cliff") {
        si->clearValidStateSamplerAllocator();
        si->setValidStateSamplerAllocator(
                [=](auto &&PH1) {
                    return ompl::MoD::DijkstraSampler::allocate(
                            std::forward<decltype(PH1)>(PH1),
                            obj_cliff,
                            {-5.0, -5.0,
                             EIGEN_PI / 4.0},
                            {19.0, 19.0, EIGEN_PI / 4.0},
                            0.25,
                            0.05);
                });
    }

    auto iter = 1000;
    auto Sampler = si->allocValidStateSampler();
    while (iter--) {
        ompl::base::State *state = si->allocState();
        Sampler->sample(state);
        printf("%lf, %lf, %lf",
               (state->as<ompl::base::CarStateSpace::StateType>())->getX(),
               (state->as<ompl::base::CarStateSpace::StateType>())->getX(),
               (state->as<ompl::base::CarStateSpace::StateType>())->getYaw());

        si->freeState(state);
    }

    return 0;
}