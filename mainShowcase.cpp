#define DEBUG // TODO activate DEBUG in PlannerSettings.h

#include <planners/ThetaStar.h>
#include "base/PlannerSettings.h"

#include "steer_functions/POSQ/POSQSteering.h"

#include "metrics/PathLengthMetric.h"

#include "planners/OMPLPlanner.hpp"
#include "planners/SmoothThetaStar.h"

#include "gui/PathEvaluation.h"

#include "PostSmoothing.h"


namespace og = ompl::geometric;

void printStats(const PathStatistics &stats)
{
    std::cout << stats.planner << std::endl;
    std::cout << "\tPath length:   \t" << stats.pathLength << std::endl;
    std::cout << "\tMax curvature: \t" << stats.curvature << std::endl;
}

int main(int argc, char **argv)
{
    PlannerSettings::steeringType = Steering::STEER_TYPE_REEDS_SHEPP;
    PlannerSettings::CarTurningRadius = 1.5;
    PlannerSettings::initializeSteering();
    PathEvaluation::initialize();

    QtVisualizer::initialize();

    std::vector<Rectangle> obstacles;
    obstacles.emplace_back(Rectangle(10, 0, 15, 14));
    obstacles.emplace_back(Rectangle(26, 10, 31, 25));

    PlannerSettings::environment = Environment::createFromObstacles(obstacles, 40, 25);
    PlannerSettings::environment->setStart(Tpoint(5, 3));
    PlannerSettings::environment->setGoal(Tpoint(36, 22));

//    PlannerSettings::environment = Environment::createRandomCorridor(50, 50, 3, 30, //1540486476); //1540445576); //1502484532); //1502407983); //1502323408); //1502316103); //1502231684); //1502227898); //1501893283); //1501892155);//1501089540); //1501089410 );//1500660612);// 1500551721);// 1500550472);
//                                                                     (unsigned int) (time(nullptr) + 123));

    Log::instantiateRun();

    for (unsigned int i = 0; i < 50; ++i) {
        PlannerSettings::environment = Environment::createRandom(50, 50, 0.1, 1542671305 + i);
        QtVisualizer::visualize(PlannerSettings::environment, 0);
        PathStatistics thetaStarStats, gripsStats, smoothThetaStarStats;

        auto *thetaStar = new ThetaStar;
        if (thetaStar->run()) {
            PostSmoothing::smoothingTime = thetaStar->planningTime();
            std::vector<Tpoint> path = thetaStar->solutionPath();
            thetaStarStats = PathEvaluation::evaluate(path, "Theta*", Qt::black);

            std::vector<GNode> trajectory = thetaStar->solutionTrajectory();
            std::vector<GNode> smoothed(trajectory);
            PostSmoothing::smooth(smoothed, path);
            auto smoothedTrajPoints = PlannerUtils::toSteeredTrajectoryPoints(smoothed);
            gripsStats = PathEvaluation::evaluate(smoothedTrajPoints, "GRIPS", Qt::red);
        } else {
            OMPL_ERROR("Theta* couldn't find a solution.");
        }

        auto *smoothThetaStar = new SmoothThetaStar;
        if (smoothThetaStar->run()) {
            std::vector<Tpoint> path = smoothThetaStar->solutionPath();
            smoothThetaStarStats = PathEvaluation::evaluate(path, "Smooth Theta*", Qt::blue);
        }

        printStats(thetaStarStats);
        std::cout << "\tSteps:\t\t\t" << thetaStar->steps() << std::endl;
        std::cout << "\tTime:\t\t\t" << thetaStar->planningTime() << std::endl;
        printStats(gripsStats);
        std::cout << "\tSteps:\t\t\t" << thetaStar->steps() << std::endl;
        std::cout << "\tTime:\t\t\t" << PostSmoothing::smoothingTime << std::endl;
        printStats(smoothThetaStarStats);
        std::cout << "\tSteps:\t\t\t" << smoothThetaStar->steps() << std::endl;
        std::cout << "\tTime:\t\t\t" << smoothThetaStar->planningTime() << std::endl;

        Log::log(nlohmann::json({
                     {"thetaStar",       {
                                                 {"curvature", thetaStarStats.curvature},
                                                 {"pathLength", thetaStarStats.pathLength},
                                                 {"steps", thetaStar->steps()},
                                                 {"time", thetaStar->planningTime()}
                                         }},
                     {"grips",           {
                                                 {"curvature", gripsStats.curvature},
                                                 {"pathLength", gripsStats.pathLength},
                                                 {"steps", thetaStar->steps()},
                                                 {"time", PostSmoothing::smoothingTime}
                                         }},
                     {"smoothThetaStar", {
                                                 {"curvature", smoothThetaStarStats.curvature},
                                                 {"pathLength", smoothThetaStarStats.pathLength},
                                                 {"steps", smoothThetaStar->steps()},
                                                 {"time", smoothThetaStar->planningTime()}
                                         }}
             }));

        delete thetaStar;
        delete smoothThetaStar;
    }

    Log::save();

    QtVisualizer::show();

    return QtVisualizer::exec();
}
