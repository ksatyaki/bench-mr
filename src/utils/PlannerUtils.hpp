#pragma once

#include <ompl/base/OptimizationObjective.h>
#include <ompl/control/PathControl.h>
#include <ompl/geometric/PathGeometric.h>

#include <cmath>
#include <iomanip>

#include "base/PlannerSettings.h"
#include "base/Primitives.h"

#if QT_SUPPORT
#include "gui/QtVisualizer.h"
#endif

namespace ob = ompl::base;
namespace og = ompl::geometric;

/**
 * Helper for common operations during planning and evaluation.
 *
 * Contains static functions for collision checking, and other path evaluations.
 */
class PlannerUtils {
 public:
  PlannerUtils() = delete;

  /**
   * Compute the angular slope between two points.
   *
   * The slope will be zero, for example, for a line from \f$(0,0)\f$ to
   * \f$(1,0)\f$. The slope will be \f$\frac{\pi}{2}\f$, for example, for a line
   * from \f$(0,0)\f$ to \f$(0,1)\f$.
   *
   * @param x1 X-coordinate of first point.
   * @param y1 Y-coordinate of first point.
   * @param x2 X-coorindate of second point.
   * @param y2 Y-coordinate of second point.
   * @returns Angular slope of the line in the interval \f$(-\pi,\pi]\f$
   */
  template <typename N>
  static double slope(const N &x1, const N &y1, const N &x2, const N &y2) {
    const auto dy = y2 - y1;
    const auto dx = x2 - x1;
    return normalizeAngle(std::atan2(dy, dx));
  }

  /**
   * \overload
   * @param a The first point.
   * @param b The second point.
   */
  static double slope(const Point &a, const Point &b) {
    const auto dy = b.y - a.y;
    const auto dx = b.x - a.x;
    return normalizeAngle(std::atan2(dy, dx));
  }

  /**
   * \overload
   * @param a The state containing the first point.
   * @param b The state containing the second point.
   */
  static double slope(const ompl::base::State *a, const ompl::base::State *b) {
    const auto dy = b->as<State>()->getY() - a->as<State>()->getY();
    const auto dx = b->as<State>()->getX() - a->as<State>()->getX();
    return normalizeAngle(std::atan2(dy, dx));
  }

  /**
   * Check whether two states are equal (up to some tolerance).
   *
   * Check compares x, y, and yaw difference. All have to be within tolerance
   * for states to be considered equal.
   * Tolerance is set in
   * PlannerSettings::GlobalSettings::OmplSettings::state_equality_tolerance.
   *
   * @param a The first state.
   * @param b The second state.
   * @returns True if the states are equal, false otherwise.
   */
  static bool equals(const ompl::base::State *a, const ompl::base::State *b) {
    const auto dy = b->as<State>()->getY() - a->as<State>()->getY();
    const auto dx = b->as<State>()->getX() - a->as<State>()->getX();
    const auto dt = b->as<State>()->getYaw() - a->as<State>()->getYaw();
    return std::abs(dx) <= global::settings.ompl.state_equality_tolerance &&
           std::abs(dy) <= global::settings.ompl.state_equality_tolerance &&
           std::abs(dt) <= global::settings.ompl.state_equality_tolerance;
  }

  static std::vector<Point> toSteeredPoints(const ob::State *a,
                                            const ob::State *b) {
    return Point::fromPath(
        og::PathGeometric(global::settings.ompl.space_info, a, b));
  }

  /**
   * Check whether points on a path collide with the environment.
   *
   * This function does not consider the collision model, since points only
   * contain x and y coordinates.
   * The environment is set through
   * PlannerSettings::GlobalSettings::environment.
   *
   * @param path The points on the path.
   * @returns True if the path collides with the environment, false otherwise.
   */
  static bool collides(const std::vector<Point> &path) {
    std::cout << "Starting.. " << std::endl;

    for (const auto &point : path) {
      if (global::settings.environment->collides(point)) {
        std::cout << "check.. " << std::endl;
        //         || global::settings.environment->bilinearDistance(path[i].x,
        //         path[i].y) < 1.5) {
        return true;
      }
    }
    return false;
  }

  /**
   * Collision check that respects the collision model.
   *
   * No additional interpolation along the path is performed prior to collision
   * checking.
   *
   * The collision model is set in
   * PlannerSettings::GlobalSettings::EnvironmentSettings::CollisionSettings::collision_model.
   * The environment is set through
   * PlannerSettings::GlobalSettings::environment.
   *
   * @param The OMPL path to check.
   * @returns True if the path collides with the environment, otherwise false.
   */
  static bool collides(const ompl::geometric::PathGeometric &path) {
    for (std::size_t i = 0; i < path.getStateCount(); ++i) {
      const auto *state = path.getState(i)->as<State>();
      if (!global::settings.environment->checkValidity(state)) return true;
    }
    return false;
  }

  /**
   * Collision check of segment between two states.
   *
   * The collision model is set in
   * PlannerSettings::GlobalSettings::EnvironmentSettings::CollisionSettings::collision_model.
   * The environment is set through
   * PlannerSettings::GlobalSettings::environment. The steering function used to
   * compute the path between the two state is defined by
   * PlannerSettings::GlobalSettings::OmplSettings::state_space.
   *
   * @param a The first state of the path segment to check.
   * @param b The last state of the path segment to check.
   * @returns True if the segment collides with the environment, otherwise
   * false.
   */
  static bool collides(const ompl::base::State *a, const ompl::base::State *b) {
#ifdef DEBUG
    OMPL_DEBUG("Checking for collision between [%f %f] and [%f %f]",
               a->as<State>()->getX(), a->as<State>()->getY(),
               b->as<State>()->getX(), b->as<State>()->getY());
    std::cout << "global::settings.ompl.state_space->validSegmentCount(a, b): "
              << global::settings.ompl.state_space->validSegmentCount(a, b)
              << std::endl;
#endif
    ompl::geometric::PathGeometric p(global::settings.ompl.space_info, a, b);
    p = interpolated(p);
    return collides(p);
  }

  /**
   * Collision check that respects the collision model.
   *
   * The path is interpolated prior to collision checking.
   *
   * The collision model is set in
   * PlannerSettings::GlobalSettings::EnvironmentSettings::CollisionSettings::collision_model.
   * The environment is set through
   * PlannerSettings::GlobalSettings::environment. Interpolation is based on the
   * state space of the path.
   *
   * @param a Deprecated.
   * @param b Deprecated.
   * @param The OMPL path to check.
   * @returns True if the path collides with the environment, otherwise false.
   */
  static bool collides(const ompl::base::State *a, const ompl::base::State *b,
                       og::PathGeometric &path) {
    // TODO this function currently behaves differently in debug mode
#ifdef DEBUG
    OMPL_DEBUG("Checking for collision between [%f %f] and [%f %f]",
               a->as<State>()->getX(), a->as<State>()->getY(),
               b->as<State>()->getX(), b->as<State>()->getY());
    path =
        ompl::geometric::PathGeometric(global::settings.ompl.space_info, a, b);
    std::cout << "global::settings.ompl.state_space->validSegmentCount(a, b): "
              << global::settings.ompl.state_space->validSegmentCount(a, b)
              << std::endl;
#endif
    path = interpolated(path);
    return collides(path);
  }

  /**
   * Check whether points on a path collide with the environment.
   *
   * This function does not consider the collision model, since points only
   * contain x and y coordinates.
   * The environment is set through
   * PlannerSettings::GlobalSettings::environment.
   *
   * @param path The points on the path.
   * @param collisions Points that collide with the environment.
   * @returns True if the path collides with the environment, false otherwise.
   */
  static bool collides(const std::vector<Point> &path,
                       std::vector<Point> &collisions) {
    collisions.clear();
    for (std::size_t i = 1; i < path.size(); ++i) {
      if (global::settings.environment->collides(path[i].x, path[i].y)) {
#if QT_SUPPORT
#ifdef DEBUG
        // QtVisualizer::drawPath(path, QColor(255, 100, 0, 170));
#endif
#endif
        collisions.emplace_back(path[i]);
        continue;
      }
    }
#ifdef DEBUG
    // QtVisualizer::drawPath(path, QColor(150, 200, 0, 70));
#endif
    return !collisions.empty();
  }

  /**
   * Collision check of segment between two states.
   *
   * The collision model is set in
   * PlannerSettings::GlobalSettings::EnvironmentSettings::CollisionSettings::collision_model.
   * The environment is set through
   * PlannerSettings::GlobalSettings::environment. The steering function used to
   * compute the path between the two state is defined by
   * PlannerSettings::GlobalSettings::OmplSettings::state_space.
   *
   * @param a The first state of the path segment to check.
   * @param b The last state of the path segment to check.
   * @param collisions Points that collide with the environment.
   * @returns True if the segment collides with the environment, otherwise
   * false.
   */
  static bool collides(const ompl::base::State *a, const ompl::base::State *b,
                       std::vector<Point> &collisions) {
    ompl::geometric::PathGeometric p(global::settings.ompl.space_info, a, b);
#if DEBUG
    std::cout << "global::settings.ompl.state_space->validSegmentCount(a, b): "
              << global::settings.ompl.state_space->validSegmentCount(a, b)
              << std::endl;
#endif
    p = interpolated(p);
    const auto path = Point::fromPath(p, false);
    return collides(path, collisions);
  }

  /**
   * Interpolate path based on its associated state space.
   *
   * @param The geometric path to interpolate.
   * @returns The interpolated path.
   */
  static ompl::geometric::PathGeometric interpolated(
      ompl::geometric::PathGeometric path) {
    if (path.getStateCount() < 2) {
#if DEBUG
      OMPL_WARN("Tried to interpolate an empty path.");
#endif
      return path;
    }
    if (path.getStateCount() > global::settings.interpolation_limit) {
#if DEBUG
      OMPL_WARN(
          "Cannot interpolate path with %d nodes (maximal %d are allowed).",
          path.getStateCount(), global::settings.interpolation_limit.value());
#endif
      return path;
    }
    if (path.length() > global::settings.max_path_length) {
#if DEBUG
      OMPL_WARN("Cannot interpolate path of length %f (maximal %f is allowed).",
                path.length(), global::settings.max_path_length.value());
#endif
      return path;
    }
#if DEBUG
    OMPL_DEBUG("Interpolating path with %d nodes and length %f.",
               path.getStateCount(), path.length());
#endif
    path.interpolate(global::settings.interpolation_limit);
    return path;
  }

  /**
   * \overload
   *
   * \param The control path to interpolate.
   */
  static ompl::control::PathControl interpolated(
      ompl::control::PathControl path) {
    if (path.getStateCount() < 2) {
#if DEBUG
      OMPL_WARN("Tried to interpolate an empty path.");
#endif
      return path;
    }
    if (path.getStateCount() > global::settings.interpolation_limit) {
#if DEBUG
      OMPL_WARN(
          "Cannot interpolate path with %d nodes (maximal %d are allowed).",
          path.getStateCount(), global::settings.interpolation_limit.value());
#endif
      return path;
    }
    if (path.length() > global::settings.max_path_length) {
#if DEBUG
      OMPL_WARN("Cannot interpolate path of length %f (maximal %f is allowed).",
                path.length(), global::settings.max_path_length.value());
#endif
      return path;
    }
#if DEBUG
    OMPL_DEBUG("Interpolating path with %d nodes and length %f.",
               path.getStateCount(), path.length());
#endif
    path.interpolate();
    return path;
  }

  static void updateAngles(ompl::geometric::PathGeometric &path,
                           bool AverageAngles = true,
                           bool preventCollisions = true) {
    if (path.getStateCount() < 2) return;

    std::vector<ob::State *> &states(path.getStates());

    double theta_old = states[0]->as<State>()->getYaw();
    states[0]->as<State>()->setYaw(slope(states[0], states[1]));
    if (preventCollisions && collides(states[0], states[1]))
      states[0]->as<State>()->setYaw(theta_old);  // revert setting
    for (int i = 1; i < path.getStateCount() - 1; ++i) {
#ifdef DEBUG
      OMPL_DEBUG("UpdateAngles: Round %d / %d", i, path.getStateCount() - 2);
#endif
      theta_old = states[i]->as<State>()->getYaw();
      if (AverageAngles) {
        double l = slope(states[i - 1], states[i]);
        double r = slope(states[i], states[i + 1]);
        //                if (std::abs(l - r) >= M_PI) {
        //                  if (l > r)
        //                    l += 2. * M_PI;
        //                  else
        //                    r += 2. * M_PI;
        //                }
        states[i]->as<State>()->setYaw((l + r) * 0.5);
      } else
        states[i]->as<State>()->setYaw(slope(states[i - 1], states[i]));

      if (preventCollisions && (collides(states[i - 1], states[i]) ||
                                collides(states[i], states[i + 1])))
        states[i]->as<State>()->setYaw(theta_old);  // revert setting
    }
    theta_old = states[path.getStateCount() - 1]->as<State>()->getYaw();
    states[path.getStateCount() - 1]->as<State>()->setYaw(slope(
        states[path.getStateCount() - 2], states[path.getStateCount() - 1]));
    if (preventCollisions && collides(states[path.getStateCount() - 1],
                                      states[path.getStateCount() - 2]))
      states[path.getStateCount() - 1]->as<State>()->setYaw(
          theta_old);  // revert setting
  }

  static void gradientDescent(ompl::geometric::PathGeometric &path,
                              unsigned int rounds, double eta,
                              double discount = 1.) {
    double dx, dy;
    for (int round = 0; round < rounds; ++round) {
      // gradient descent along distance field, excluding start/end nodes
      for (int i = 1; i < path.getStateCount() - 1; ++i) {
        // compute gradient
        global::settings.environment->distanceGradient(
            path.getStates()[i]->as<State>()->getX(),
            path.getStates()[i]->as<State>()->getY(), dx, dy, 1.);
        double distance = global::settings.environment->bilinearDistance(
            path.getStates()[i]->as<State>()->getX(),
            path.getStates()[i]->as<State>()->getY());
        distance = std::max(.1, distance);
        path.getStates()[i]->as<State>()->setX(
            path.getStates()[i]->as<State>()->getX() - eta * dx / distance);
        path.getStates()[i]->as<State>()->setY(
            path.getStates()[i]->as<State>()->getY() + eta * dy / distance);
      }
      eta *= discount;
    }
  }

  static void gradientDescent(std::vector<Point> &path, unsigned int rounds,
                              double eta, double discount = 1.) {
    double dx, dy;
    for (int round = 0; round < rounds; ++round) {
      // gradient descent along distance field, excluding start/end nodes
      for (int i = 1; i < path.size() - 1; ++i) {
        // compute gradient
        global::settings.environment->distanceGradient(path[i].x, path[i].y, dx,
                                                       dy, 1.);
        double distance = global::settings.environment->bilinearDistance(
            path[i].x, path[i].y);
        distance = std::max(.1, distance);
        path[i].x -= eta * dx / distance;
        path[i].y += eta * dy / distance;
      }
      eta *= discount;
    }
  }

  /**
   * Linearly interpolate between two points.
   *
   * @param a The first point.
   * @param b The last point.
   * @param dt The step to take (where \f$t\f$ parametrizes the line, i.e.,
   * \f$t\in[0,1]\f$).
   * @returns The points on the line from \p a to \p b, including both of these
   * points.
   */
  static std::vector<Point> linearInterpolate(const Point &a, const Point &b,
                                              double dt = 0.1) {
    std::vector<Point> points;
    double dx = (b.x - a.x);
    double dy = (b.y - a.y);
    const double size = std::sqrt(dx * dx + dy * dy);
    if (size == 0) return std::vector<Point>{a};
    dx = dx / size * dt;
    dy = dy / size * dt;
    const auto steps = (int)(size / std::sqrt(dx * dx + dy * dy));

    for (int j = 1; j <= steps; ++j)
      points.emplace_back(a.x + dx * j, a.y + dy * j);

    if (points.empty()) points.emplace_back(a);

    return points;
  }

  /**
   * \overload
   *
   * This does not take into account yaw and returns a list of xy points.
   *
   * @param a The first state.
   * @param b The last state.
   * @param dt The step to take (where \f$t\f$ parametrizes the line, i.e.,
   * \f$t\in[0,1]\f$).
   * @returns The points on the line from \p a to \p b, including both of these
   * points.
   */
  static std::vector<Point> linearInterpolate(ompl::base::State *a,
                                              ompl::base::State *b,
                                              double dt = 0.1) {
    return linearInterpolate(
        Point(a->as<State>()->getX(), a->as<State>()->getY()),
        Point(b->as<State>()->getX(), b->as<State>()->getY()), dt);
  }

  /**
   * Find closest collision-free point in an ordered set of points.
   *
   * The closest point will be converted to a state, whose yaw is computed
   * based on neighboring points.
   *
   * @param x The reference point.
   * @param points The collection of points to check.
   * @returns The state closest to \p x.
   */
  static ompl::base::State *closestPoint(Point x,
                                         const std::vector<Point> &points) {
    if (points.size() == 1) return points[0].toState();
    unsigned int closest = 0;
    double dist = points[closest].distanceSquared(x);
    for (unsigned int i = 1; i < points.size() - 1; ++i) {
      if (global::settings.environment->collides(points[i].x, points[i].y))
        continue;
      const double d = points[i].distanceSquared(x);
      if (d < dist) {
        dist = d;
        closest = i;
      }
    }
    double theta;
    if (closest == 0)
      theta = slope(points[0], points[1]);
    else
      theta = slope(points[closest - 1], points[closest + 1]);
    return points[closest].toState(theta);
  }

  /**
   * Compute total length of a path.
   *
   * The length between two points is calculated as Euclidean distance.
   * The total length is computed as the sum of pairwise distances.
   *
   * @param path The ordered points along the path.
   * @returns The total length of the path as the sum of pairwise Euclidean
   * distances.
   */
  static double totalLength(const std::vector<Point> &path) {
    double l = 0;
    for (size_t i = 1; i < path.size(); ++i) {
      l += path[i].distance(path[i - 1]);
    }
    return l;
  }

  static double totalCost(const ompl::geometric::PathGeometric &trajectory) {
    double c(0.0);
    for (size_t i = 1; i < trajectory.getStateCount(); ++i) {
      c = c +
          (global::settings.ompl.objective)
              ->motionCost(trajectory.getState(i - 1), trajectory.getState(i))
              .value();
    }
    return c;
  }

  static double totalCost(const ompl::control::PathControl &trajectory) {
    double c(0.0);
    for (size_t i = 1; i < trajectory.getStateCount(); ++i) {
      c = c +
          (global::settings.ompl.objective)
              ->motionCost(trajectory.getState(i - 1), trajectory.getState(i))
              .value();
    }
    return c;
  }

  /**
   * Compute equidistance samples along path.
   *
   * @param path The input path.
   * @param targetSize The number of equdistant samples on the output path.
   * @returns The path of size targetSize with equidistant samples.
   */
  static std::vector<Point> equidistantSampling(const std::vector<Point> &path,
                                                size_t targetSize) {
    std::vector<Point> result;
    const double total = totalLength(path);
    const double targetSegment = total / targetSize;
    result.emplace_back(path[0]);
    double sofar = 0;
    double segment = 0;
    size_t resultCounter = 1;
    for (size_t i = 1; i < path.size(); ++i) {
      double dx = (path[i].x - path[i - 1].x);
      double dy = (path[i].y - path[i - 1].y);
      const double l = std::sqrt(dx * dx + dy * dy);
      if (std::abs(l) < 1e-3) continue;
      dx /= l;
      dy /= l;
      if (segment + l < targetSegment) {
        segment += l;
        OMPL_DEBUG("EquidistantSampling: Segment too short between %i and %i",
                   i - 1, i);
        continue;
      }
      double start = std::fmod(segment + l, targetSegment) - segment;
      segment = 0;
      const auto segmentSteps =
          static_cast<unsigned int>(std::floor((segment + l) / targetSegment));
      for (unsigned int j = 0; j < segmentSteps; ++j) {
        const double alpha = start + j * targetSegment;
        const Point p(path[i - 1].x + dx * alpha, path[i - 1].y + dy * alpha);
        result.emplace_back(p);
      }
      sofar += segmentSteps * targetSegment;
      segment = l - segmentSteps * targetSegment;
    }
    result.emplace_back(path.back());
    return result;
  }

  /**
   * Convert a number to a string with the given precision.
   *
   * @param v The number to convert to a string.
   * @param precision The precision used when converting the number to a string.
   * @returns The string containing the number.
   */
  template <typename N>
  static std::string num2str(const N &v, unsigned int precision = 3) {
    std::stringstream stream;
    stream << std::fixed << std::setprecision(precision) << v;
    return stream.str();
  }

  /**
   * Normalize angle in radians to the interval \f$(-\pi, \pi]\f$.
   *
   * @param angle The angle to normalize.
   * @returns The normalized angle.
   */
  inline static double normalizeAngle(double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
  }
};
