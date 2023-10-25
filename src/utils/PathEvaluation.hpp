#pragma once

#include <metrics/AOLMetric.h>
#include <metrics/ClearingMetric.h>
#include <metrics/MaxCurvatureMetric.h>
#include <metrics/NormalizedCurvatureMetric.h>
#include <metrics/PathLengthMetric.h>
#include <metrics/TotalCostMetric.h>
#include <smoothers/chomp/CHOMP.h>

#include <smoothers/ompl/OmplSmoother.hpp>

#include "base/PathStatistics.hpp"
#include "base/PlannerConfigurator.hpp"
#include "planners/AbstractPlanner.h"
#include "smoothers/grips/GRIPS.h"
#include "utils/Log.h"

struct PathEvaluation {
 private:
  /**
   * Creates an empty entry for the given planner in the JSON info object. Used
   * in cases where the planner failed to find a solution or an error was
   * encountered.
   */
  static void createEmptyEntry(const std::string &planner_name,
                               nlohmann::json &info) {
    static PathStatistics empty_stats;
    auto &j = info["plans"][planner_name];
    j["path"] = {};
    j["stats"] = nlohmann::json(empty_stats)["stats"];
    j["trajectory"] = {};
    j["intermediary_solutions"] = {};
    j["params"] = {};
  }

 public:
  /**
   * Identifies cusps in a solution path by comparing the yaw angles between
   * every second state.
   */
  static void computeCusps(PathStatistics &stats,
                           const std::vector<Point> path) {
    std::vector<Point> &cusps = stats.cusps.value();

    auto prev = path.begin();
    auto current = prev;
    auto next = prev;
    while (next != path.end()) {
      // advance until current point != prev point, i.e., skip duplicates
      if (prev->distance(*current) <= 0) {
        ++current;
        ++next;
      } else if (current->distance(*next) <= 0) {
        ++next;
      } else {
        const double yaw_prev = PlannerUtils::slope(*prev, *current);
        const double yaw_next = PlannerUtils::slope(*current, *next);

        // compute angle difference in [0, pi)
        // close to pi -> cusp; 0 -> straight line; inbetween -> curve
        const double yaw_change =
            std::abs(PlannerUtils::normalizeAngle(yaw_next - yaw_prev));

        if (yaw_change > global::settings.cusp_angle_threshold) {
          cusps.emplace_back(*current);
        }
        prev = current;
        current = next;
        ++next;
      }
    }
  }

  static bool evaluate(PathStatistics &stats,
                       const ompl::control::PathControl &path,
                       const AbstractPlanner *planner) {
    stats.planning_time = planner->planningTime();
    stats.collision_time = global::settings.environment->elapsedCollisionTime();
    stats.steering_time = global::settings.ompl.steering_timer.elapsed();
    stats.planner = planner->name();
    stats.planner_settings = planner->getSettings();
    if (path.getStateCount() < 2) {
      stats.path_found = false;
      stats.exact_goal_path = false;
    } else {
      stats.path_found = true;
      auto solution = path;
      solution = PlannerUtils::interpolated(path);
      stats.path_collides = !planner->isValid(solution, stats.collisions);
      stats.exact_goal_path =
          Point(solution.getStates().back())
              .distance(global::settings.environment->goal()) <=
          global::settings.exact_goal_radius;
      stats.path_length = PathLengthMetric::evaluate(solution);
      stats.total_cost = TotalCostMetric::evaluate(solution);
      stats.max_curvature = MaxCurvatureMetric::evaluate(solution);
      stats.normalized_curvature =
          NormalizedCurvatureMetric::evaluate(solution);
      stats.aol = AOLMetric::evaluate(solution);
      // This is not implemented in OMPL for ompl::control
      // stats.smoothness = solution.smoothness();

      if (global::settings.evaluate_clearing &&
          global::settings.environment->distance(0., 0.) >= 0.) {
        const auto clearings = ClearingMetric::clearingDistances(solution);
        stats.mean_clearing_distance = stat::mean(clearings);
        stats.median_clearing_distance = stat::median(clearings);
        stats.min_clearing_distance = stat::min(clearings);
        stats.max_clearing_distance = stat::max(clearings);
      }
      const auto p = Point::fromPath(solution);
      computeCusps(stats, p);
    }
    return stats.path_found;
  }

  static bool evaluate(PathStatistics &stats,
                       const ompl::geometric::PathGeometric &path,
                       const AbstractPlanner *planner) {
    stats.planning_time = planner->planningTime();
    stats.collision_time = global::settings.environment->elapsedCollisionTime();
    stats.steering_time = global::settings.ompl.steering_timer.elapsed();
    stats.planner = planner->name();
    stats.planner_settings = planner->getSettings();
    if (path.getStateCount() < 2) {
      stats.path_found = false;
      stats.exact_goal_path = false;
    } else {
      stats.path_found = true;
      auto solution = path;

      // assume if SBPL has found a solution, it does not collide and is exact
      if (planner->name().rfind("SBPL", 0) == 0) {
        // do not interpolate the path returned by SBPL (it uses its own steer
        // function)
        stats.path_collides = false;
        stats.exact_goal_path = true;
      } else {
        solution = PlannerUtils::interpolated(path);
        stats.path_collides = !planner->isValid(solution, stats.collisions);
        stats.exact_goal_path =
            Point(solution.getStates().back())
                .distance(global::settings.environment->goal()) <=
            global::settings.exact_goal_radius;
      }
      stats.path_length = PathLengthMetric::evaluate(solution);
      stats.total_cost = TotalCostMetric::evaluate(solution);
      stats.max_curvature = MaxCurvatureMetric::evaluate(solution);
      stats.normalized_curvature =
          NormalizedCurvatureMetric::evaluate(solution);
      stats.aol = AOLMetric::evaluate(solution);
      stats.smoothness = solution.smoothness();

      if (global::settings.evaluate_clearing &&
          global::settings.environment->distance(0., 0.) >= 0.) {
        const auto clearings = ClearingMetric::clearingDistances(solution);
        stats.mean_clearing_distance = stat::mean(clearings);
        stats.median_clearing_distance = stat::median(clearings);
        stats.min_clearing_distance = stat::min(clearings);
        stats.max_clearing_distance = stat::max(clearings);
      }

      const auto p = Point::fromPath(solution);
      computeCusps(stats, p);
    }
    return stats.path_found;
  }

  template <class PLANNER>
  static bool evaluate(PLANNER &planner, nlohmann::json &info) {
    PathStatistics stats(planner.name());
    auto &j = info["plans"][planner.name()];
    OMPL_INFORM(("Running " + planner.name() + "...").c_str());
    bool success;
    global::settings.environment->resetCollisionTimer();
    global::settings.ompl.steering_timer.reset();
    try {
      if (planner.run()) {
        success = PathEvaluation::evaluate(stats, planner.solution(), &planner);
        j["path"] = Log::serializeTrajectory(planner.solution(), false);
      } else {
        createEmptyEntry(planner.name(), info);
        std::cout << "<stats> No solution was found. </stats>\n";
        return false;
      }
    } catch (std::bad_alloc &ba) {
      // we ran out of memory
      OMPL_ERROR("<stats> Error </stats>\nPlanner %s ran out of memory: %s.",
                 planner.name().c_str(), ba.what());
      createEmptyEntry(planner.name(), info);
      return false;
    } catch (ompl::Exception &ex) {
      OMPL_ERROR("Unable to evaluate new planner %s.\n%s",
                 planner.name().c_str(), ex.what());
      createEmptyEntry(planner.name(), info);
      return false;
    } catch (std::exception &ex) {
      OMPL_ERROR("GENERIC ERROR: %s.", ex.what());
      return false;
    }
    catch (...) {
      OMPL_ERROR(
          "<stats> Error </stats>\nAn unknown exception occurred while running "
          "planner %s.",
          planner.name().c_str());
      createEmptyEntry(planner.name(), info);
      return false;
    }

    std::cout << stats << std::endl;
    std::cout << "Steer function: "
              << Steering::to_string(global::settings.steer.steering_type)
              << std::endl;
    // do not interpolate SBPL solutions since they do not use OMPL steer
    // functions
    if (planner.name().rfind("SBPL", 0) == 0) {
      j["trajectory"] = Log::serializeTrajectory(planner.solution(), false);
    } else {
      j["trajectory"] = Log::serializeTrajectory(planner.solution());
    }
    j["stats"] = nlohmann::json(stats)["stats"];

    // add intermediary solutions
    std::vector<nlohmann::json> intermediaries;
    for (const auto &is : planner.intermediarySolutions) {
      PathStatistics is_stats;
      evaluate(is_stats, is.solution, &planner);
      nlohmann::json s{{"time", is.time},
                       {"collision_time", is_stats.collision_time},
                       {"steering_time", is_stats.steering_time},
                       {"cost", is.cost},
                       {"trajectory", Log::serializeTrajectory(is.solution)},
                       {"path", Log::serializeTrajectory(is.solution, false)},
                       {"stats", nlohmann::json(is_stats)["stats"]}};
      intermediaries.emplace_back(s);
    }
    j["intermediary_solutions"] = intermediaries;
    return success;
  }

  template <class PLANNER>
  static bool evaluate(nlohmann::json &info) {
    PLANNER *planner = nullptr;
    try {
      planner = new PLANNER;
      PlannerConfigurator::configure(*planner);
    } catch (std::bad_alloc &ba) {
      // we ran out of memory
      OMPL_ERROR(
          "<stats> Error </stats>\nRan out of memory while creating planner "
          "%s: %s.",
          AbstractPlanner::LastCreatedPlannerName.c_str(), ba.what());
      createEmptyEntry(AbstractPlanner::LastCreatedPlannerName, info);
      delete planner;
      return false;
    } catch (ompl::Exception &ex) {
      OMPL_ERROR("Unable to create new planner %s.\n%s",
                 AbstractPlanner::LastCreatedPlannerName.c_str(), ex.what());
      createEmptyEntry(AbstractPlanner::LastCreatedPlannerName, info);
      delete planner;
      return false;
    } catch (...) {
      OMPL_ERROR(
          "<stats> Error </stats>\nAn unknown exception occurred while "
          "creating planner %s.",
          AbstractPlanner::LastCreatedPlannerName.c_str());
      createEmptyEntry(AbstractPlanner::LastCreatedPlannerName, info);
      delete planner;
      return false;
    }
    auto result = evaluate(*planner, info);
    delete planner;
    return result;
  }

  template <class PLANNER>
  static bool evaluateSmoothers(nlohmann::json &info) {
    PLANNER *planner = nullptr;
    try {
      planner = new PLANNER;
      PlannerConfigurator::configure(*planner);
    } catch (std::bad_alloc &ba) {
      // we ran out of memory
      OMPL_ERROR(
          "<stats> Error </stats>\nRan out of memory while creating planner "
          "%s: %s.",
          AbstractPlanner::LastCreatedPlannerName.c_str(), ba.what());
      createEmptyEntry(AbstractPlanner::LastCreatedPlannerName, info);
      delete planner;
      return false;
    } catch (...) {
      OMPL_ERROR(
          "<stats> Error </stats>\nAn unknown exception occurred while "
          "creating planner %s.",
          AbstractPlanner::LastCreatedPlannerName.c_str());
      createEmptyEntry(AbstractPlanner::LastCreatedPlannerName, info);
      delete planner;
      return false;
    }
    if (!evaluate<PLANNER>(*planner, info)) {
      OMPL_WARN("Cannot evaluate smoothers since no solution could be found.");
      delete planner;
      return false;
    }
    auto &j = info["plans"][planner->name()]["smoothing"];
    global::settings.environment->resetCollisionTimer();
    global::settings.ompl.steering_timer.reset();

    if (global::settings.benchmark.smoothing.grips) {
      const double cached_min_node_dist =
          global::settings.smoothing.grips.min_node_distance;
      if (global::settings.steer.steering_type ==
          Steering::STEER_TYPE_CC_DUBINS) {
        // XXX increase min distance between vertices to ensure GRIPS can steer
        // using CC Dubins
        global::settings.smoothing.grips.min_node_distance = 40.;
      }
      // GRIPS
      og::PathGeometric grips(planner->solution());
      GRIPS::smooth(grips);
      PathStatistics grips_stats;
      evaluate(grips_stats, grips, planner);
      j["grips"] = {
          {"time", GRIPS::smoothingTime},
          {"collision_time",
           global::settings.environment->elapsedCollisionTime()},
          {"steering_time", global::settings.ompl.steering_timer.elapsed()},
          {"name", "GRIPS"},
          {"inserted_nodes", GRIPS::insertedNodes},
          {"pruning_rounds", GRIPS::pruningRounds},
          {"cost", grips.length()},
          {"trajectory", Log::serializeTrajectory(grips)},
          {"path", Log::serializeTrajectory(grips, false)},
          {"stats", nlohmann::json(grips_stats)["stats"]},
          {"round_stats", GRIPS::statsPerRound}};
      global::settings.smoothing.grips.min_node_distance = cached_min_node_dist;
    }
    if (global::settings.benchmark.smoothing.chomp) {
      global::settings.environment->resetCollisionTimer();
      global::settings.ompl.steering_timer.reset();
      // CHOMP
      CHOMP chomp;
      chomp.run(planner->solution());
      PathStatistics chomp_stats;
      evaluate(chomp_stats, chomp.solution(), planner);
      j["chomp"] = {
          {"time", chomp.planningTime()},
          {"collision_time",
           global::settings.environment->elapsedCollisionTime()},
          {"steering_time", global::settings.ompl.steering_timer.elapsed()},
          {"name", "CHOMP"},
          {"cost", chomp.solution().length()},
          {"path", Log::serializeTrajectory(chomp.solution(), false)},
          {"trajectory", chomp.solutionPath()},
          {"stats", nlohmann::json(chomp_stats)["stats"]}};
    }

    // OMPL Smoothers
    OmplSmoother smoother(planner->simpleSetup(), planner->solution());
    if (global::settings.benchmark.smoothing.ompl_shortcut) {
      global::settings.environment->resetCollisionTimer();
      global::settings.ompl.steering_timer.reset();
      // Shortcut
      PathStatistics stats;
      TimedResult tr = smoother.shortcutPath();
      evaluate(stats, tr.trajectory, planner);
      j["ompl_shortcut"] = {
          {"time", tr.elapsed()},
          {"collision_time",
           global::settings.environment->elapsedCollisionTime()},
          {"steering_time", global::settings.ompl.steering_timer.elapsed()},
          {"name", "Shortcut"},
          {"cost", tr.trajectory.length()},
          {"path", Log::serializeTrajectory(tr.trajectory, false)},
          {"trajectory", Log::serializeTrajectory(tr.trajectory)},
          {"stats", nlohmann::json(stats)["stats"]}};
    }
    if (global::settings.benchmark.smoothing.ompl_bspline) {
      global::settings.environment->resetCollisionTimer();
      global::settings.ompl.steering_timer.reset();
      // B-Spline
      PathStatistics stats;
      TimedResult tr = smoother.smoothBSpline();
      evaluate(stats, tr.trajectory, planner);
      j["ompl_bspline"] = {
          {"time", tr.elapsed()},
          {"collision_time",
           global::settings.environment->elapsedCollisionTime()},
          {"steering_time", global::settings.ompl.steering_timer.elapsed()},
          {"name", "B-Spline"},
          {"cost", tr.trajectory.length()},
          {"path", Log::serializeTrajectory(tr.trajectory, false)},
          {"trajectory", Log::serializeTrajectory(tr.trajectory)},
          {"stats", nlohmann::json(stats)["stats"]}};
    }
    if (global::settings.benchmark.smoothing.ompl_simplify_max) {
      global::settings.environment->resetCollisionTimer();
      global::settings.ompl.steering_timer.reset();
      // Simplify Max
      PathStatistics stats;
      TimedResult tr = smoother.simplifyMax();
      evaluate(stats, tr.trajectory, planner);
      j["ompl_simplify_max"] = {
          {"time", tr.elapsed()},
          {"collision_time",
           global::settings.environment->elapsedCollisionTime()},
          {"steering_time", global::settings.ompl.steering_timer.elapsed()},
          {"name", "SimplifyMax"},
          {"cost", tr.trajectory.length()},
          {"path", Log::serializeTrajectory(tr.trajectory, false)},
          {"trajectory", Log::serializeTrajectory(tr.trajectory)},
          {"stats", nlohmann::json(stats)["stats"]}};
    }

    delete planner;
    return true;
  }

  /**
   * Evaluates an anytime path planner by running the planner for each of the
   * provided time intervals (in seconds). This method populates the
   * "intermediary_solutions" field of the JSON object for the given planner.
   */
  // TODO remove? Python front-end adds this functionality
  template <class PLANNER>
  static bool evaluateAnytime(nlohmann::json &info) {
    PLANNER planner;
    const std::vector<double> times =
        global::settings.benchmark.anytime_intervals;
    PathStatistics stats(planner.name());
    auto &j = info["plans"][planner.name()];
    std::vector<nlohmann::json> intermediaries;
    bool success = false;
    const double cached_time_limit = global::settings.max_planning_time;
    for (double time : times) {
      OMPL_INFORM(("Running " + planner.name() + " for " +
                   std::to_string(time) + "s...")
                      .c_str());
      global::settings.max_planning_time = time;
      global::settings.environment->resetCollisionTimer();
      if (planner.run()) {
        success = PathEvaluation::evaluate(stats, planner.solution(), &planner);
        j["path"] = Log::serializeTrajectory(planner.solution(), false);
      } else {
        j["path"] = {};
      }
      std::cout << stats << std::endl;
      std::cout << "Steer function: "
                << Steering::to_string(global::settings.steer.steering_type)
                << std::endl;

      // add intermediary solutions
      nlohmann::json s{
          {"time", planner.planningTime()},
          {"collision_time",
           global::settings.environment->elapsedCollisionTime()},
          {"max_time", time},
          {"cost", stats.path_length},
          {"trajectory", Log::serializeTrajectory(planner.solution())},
          {"path", Log::serializeTrajectory(planner.solution(), false)},
          {"stats", nlohmann::json(stats)["stats"]}};
      intermediaries.emplace_back(s);
    }

    j["intermediary_solutions"] = intermediaries;
    if (planner.name().rfind("SBPL", 0) == 0) {
      j["trajectory"] = Log::serializeTrajectory(planner.solution(), false);
    } else {
      j["trajectory"] = Log::serializeTrajectory(planner.solution());
    }
    j["stats"] = nlohmann::json(stats)["stats"];
    // restore global time limit
    global::settings.max_planning_time = cached_time_limit;
    return success;
  }

  PathEvaluation() = delete;
};
