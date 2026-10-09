#ifndef CAR_SWARM_AGENT_TRAJECTORY_YIELD_HPP
#define CAR_SWARM_AGENT_TRAJECTORY_YIELD_HPP

#include <string>
#include <optional>
#include <vector>

#include "car_swarm_agent/trajectory_collision.hpp"
#include "car_swarm_agent/continuous_trajectory.hpp"

namespace car_swarm_agent {

enum class YieldStatus
{
  Unchanged, Yielded, AwaitingPeerYield, InformationInsufficient, NoFeasiblePlan
};

struct YieldOptions
{
  std::int64_t sample_step_ns{100000000};
  std::int64_t wait_step_ns{500000000};
  std::int64_t max_wait_ns{10000000000LL};
};

struct YieldResult
{
  YieldStatus status{YieldStatus::InformationInsufficient};
  // Only Unchanged/Yielded carry a fully checked trajectory. Never fall back to
  // the original conflicting trajectory when this optional is empty.
  std::optional<msg::PredictedTrajectory> trajectory;
  std::optional<TrajectoryConflict> original_conflict;
  std::optional<CollisionCheckResult> verification;
  std::optional<std::size_t> stop_point_index;
  std::int64_t wait_ns{0};
  std::size_t candidates_tried{0};
};

// Pure fixed-path timing baseline, not S-T optimization or a braking controller.
// Larger vehicle_id yields (std::string lexicographic order). IDs must be distinct
// and nonempty; use zero-padded numeric suffixes for numerical ID ordering.
// now_ns must share the trajectories' clock. The neighbor must cover the entire
// original/candidate interval; expired or incomplete coverage is insufficient.
// Starts at time_from_start=0/STOP. Searches increasing wait durations, and for
// each duration searches original points from nearest-to-conflict backwards.
// Only points strictly before the first sampled conflict and at/after now_ns
// are eligible. Inserts a duplicate STOP endpoint without changing spatial poses,
// stamps, or incoming directions of subsequent original points.
// Invalid input throws std::invalid_argument. Safety is only at sample precision.
YieldResult planFixedPathYield(
  const msg::PredictedTrajectory & own,
  const msg::PredictedTrajectory & neighbor,
  std::int64_t now_ns,
  const PlannerConfig & own_geometry = {},
  const PlannerConfig & neighbor_geometry = {},
  const YieldOptions & options = {});

// Acceleration-constrained extension of the fixed-path yielding module.

enum class STStatus { Success, NoFeasiblePlan, InformationInsufficient, SearchLimit };

struct STOptions
{
  ContinuousCheckOptions continuous_check{};

  double initial_speed{1.0};
  double max_speed{2.0};
  double max_acceleration{1.0};

  std::int64_t time_step_ns{500000000};
  std::int64_t sample_step_ns{50000000};

  int max_steps{80};
  std::size_t max_nodes{50000};

  double s_resolution{0.1};
  double v_resolution{0.1};
};

struct STSample
{
  std::int64_t time_from_start_ns{0};
  double s{0.0};
  double speed{0.0};
};

struct STResult
{
  STStatus status{STStatus::NoFeasiblePlan};

  std::optional<msg::PredictedTrajectory> trajectory;
  std::vector<STSample> samples;
  std::optional<CollisionCheckResult> verification;
  std::size_t nodes_expanded{0};

  std::optional<ContinuousTrajectory> continuous;
  std::optional<ContinuousCheckResult> continuous_verification;
};

// Fixed Hybrid A* path: Pose[i].signed_distance/curvature describe incoming arcs.
// start_header.stamp is both the planning start time and output time origin.
// Explicitly supply the measured initial speed; the final speed is zero.
// Each gear change is a mandatory zero-speed boundary. Every edge and the full
// result are checked by detectTrajectoryCollision. Unknown prediction coverage
// is never safe. Throws std::invalid_argument for malformed inputs.
// This is a bounded, pruned search baseline, not a controller or completeness
// guarantee; NoFeasiblePlan means no plan found with these search settings.
STResult planSTSpeed(
  const std::vector<Pose> & path,
  const msg::PredictedTrajectory & neighbor,
  const std_msgs::msg::Header & start_header,
  const std::string & vehicle_id,
  const PlannerConfig & own_geometry = {},
  const PlannerConfig & neighbor_geometry = {},
  const STOptions & options = {});
}  // namespace car_swarm_agent
#endif
