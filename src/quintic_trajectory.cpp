#include "car_swarm_agent/quintic_trajectory.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace car_swarm_agent {
namespace {
double derivative(const std::array<double, 6> & c, double u, int d, double h)
{
  double value = 0;
  for (int k = 5; k >= d; --k) {
    double f = 1;
    for (int j = 0; j < d; ++j) {f *= k - j;}
    value = value * u + c[k] * f;
  }
  return value / std::pow(h, d);
}
double stopCurvature(const QuinticPiece & p, double u)
{
  // Factor out the known zero of velocity. Direct cross(v,a)/|v|^3 loses
  // precision near a stop, especially at u=1 where Horner terms cancel.
  const bool end = u > 0.5;
  std::array<std::array<double, 2>, 4> q;
  for (int j = 0; j < 4; ++j) {
    const int d = j + 2;
    double factorial = 1;
    for (int k = 2; k <= d; ++k) {factorial *= k;}
    const double sign = end && d % 2 ? -1 : 1;
    q[j] = {sign * derivative(p.x, end ? 1 : 0, d, 1) / factorial * d,
      sign * derivative(p.y, end ? 1 : 0, d, 1) / factorial * d};
  }
  const auto cross = [&](int a, int b) {
      return q[a][0] * q[b][1] - q[a][1] * q[b][0];
    };
  const double w = end ? 1 - u : u;
  double x = 0, y = 0;
  for (int j = 3; j >= 0; --j) {x = x * w + q[j][0]; y = y * w + q[j][1];}
  const double norm = std::hypot(x, y);
  const double constant = cross(0, 1);
  const double numerator = 2 * cross(0, 2) +
    (3 * cross(0, 3) + cross(1, 2)) * w + 2 * cross(1, 3) * w * w +
    cross(2, 3) * w * w * w;
  // The optimizer enforces tangent jerk at a stop. Remove only its numerical
  // equality residual; a material nonzero constant implies singular steering.
  const double equality_tolerance = 1e-9 * std::max(1.0,
    std::hypot(q[0][0], q[0][1]) * std::hypot(q[1][0], q[1][1]));
  const double residual = std::abs(constant) <= equality_tolerance ? 0 :
    constant / std::max(w, 1e-16);
  if (norm < 1e-12) {return std::abs(numerator + residual) < 1e-12 ? 0 : INFINITY;}
  return p.gear * (end ? -1 : 1) * (numerator + residual) / std::pow(norm, 3);
}
std::pair<std::size_t, double> locate(
  const std::vector<QuinticPiece> & pieces, double t, bool left)
{
  if (!std::isfinite(t) || t < 0) {throw std::invalid_argument("Invalid polynomial time");}
  double start = 0;
  for (std::size_t i = 0; i < pieces.size(); ++i) {
    double end = start + pieces[i].duration;
    if (t < end || (left && t == end) || i + 1 == pieces.size()) {
      if (t > end + 1e-8) {throw std::out_of_range("Polynomial extrapolation");}
      return {i, std::clamp((t - start) / pieces[i].duration, 0.0, 1.0)};
    }
    start = end;
  }
  throw std::out_of_range("Polynomial time");
}
}  // namespace

QuinticTrajectory::QuinticTrajectory(
  std::vector<QuinticPiece> pieces, std_msgs::msg::Header header)
: pieces_(std::move(pieces)), header_(std::move(header))
{
  if (pieces_.empty() || header_.frame_id.empty() || header_.stamp.nanosec >= 1000000000) {
    throw std::invalid_argument("Empty polynomial/header");
  }
  for (const auto & p : pieces_) {
    if (!std::isfinite(p.duration) || p.duration <= 0 ||
      std::abs(p.gear) > 1 || !std::isfinite(p.stopped_yaw)) {
      throw std::invalid_argument("Invalid polynomial piece");
    }
    for (auto c : {p.x, p.y}) {
      for (double x : c) {
        if (!std::isfinite(x)) {throw std::invalid_argument("Nonfinite coefficient");}
      }
      if (p.gear == 0) {
        for (int i = 1; i < 6; ++i) {
          if (std::abs(c[i]) > 1e-9) {throw std::invalid_argument("Moving hold");}
        }
      }
    }
  }
  double boundary = 0;
  for (std::size_t i = 1; i < pieces_.size(); ++i) {
    boundary += pieces_[i - 1].duration;
    if (pieces_[i - 1].gear != pieces_[i].gear &&
      std::abs(std::remainder(evaluate(boundary, true).pose.yaw -
      evaluate(boundary).pose.yaw, 6.28318530717958647692)) > 1e-4)
    {throw std::invalid_argument("Heading jump at stopped gear boundary");}
    const auto & a = pieces_[i - 1]; const auto & b = pieces_[i];
    for (int d = 0; d < 3; ++d) {
      for (int axis = 0; axis < 2; ++axis) {
        const double l = derivative(axis ? a.y : a.x, 1, d, a.duration);
        const double r = derivative(axis ? b.y : b.x, 0, d, b.duration);
        // Holds and gear changes require position/velocity continuity, while
        // acceleration may have a finite jump at the mandatory stop.
        if (d < 2 || a.gear == b.gear) {
          if (std::abs(l - r) > 1e-5) {throw std::invalid_argument("Polynomial discontinuity");}
        }
        if (d == 1 && a.gear != b.gear && (std::abs(l) > 1e-6 || std::abs(r) > 1e-6)) {
          throw std::invalid_argument("Nonzero gear-change speed");
        }
      }
    }
  }
  if (evaluate(duration(), true).speed > 1e-6) {
    throw std::invalid_argument("Polynomial must end stopped");
  }
}

double QuinticTrajectory::duration() const
{
  double result = 0; for (const auto & p : pieces_) {result += p.duration;} return result;
}
ContinuousState QuinticTrajectory::evaluate(double t, bool left) const
{
  auto [i, u] = locate(pieces_, t, left);
  const auto & p = pieces_[i];
  const double vx = derivative(p.x, u, 1, p.duration);
  const double vy = derivative(p.y, u, 1, p.duration);
  double ax = derivative(p.x, u, 2, p.duration);
  double ay = derivative(p.y, u, 2, p.duration);
  const double speed = std::hypot(vx, vy);
  double yaw = p.stopped_yaw, k = 0;
  if (speed > 1e-9) {
    yaw = std::atan2(p.gear * vy, p.gear * vx);
    k = p.gear * (vx * ay - vy * ax) / std::pow(speed, 3);
  }
  if (p.gear != 0 && speed < 1e-4 &&
    std::hypot(derivative(p.x, u > 0.5 ? 1 : 0, 1, p.duration),
    derivative(p.y, u > 0.5 ? 1 : 0, 1, p.duration)) < 1e-8)
  {
    k = stopCurvature(p, u);
  }
  if (speed <= 1e-9 && p.gear != 0) {
    // Recover the one-sided body heading at a stop from the first nonzero
    // velocity Taylor term, rather than atan2 of cancellation residuals.
    double dx = ax, dy = ay;
    if (std::hypot(dx, dy) > 1e-8) {
      const double sign = u > 0.5 ? -1.0 : 1.0; dx *= sign; dy *= sign;
    } else {
      dx = derivative(p.x, u, 3, p.duration);
      dy = derivative(p.y, u, 3, p.duration);
    }
    if (std::hypot(dx, dy) > 1e-8) {yaw = std::atan2(p.gear * dy, p.gear * dx);}
  }
  const int gear = p.gear == 0 ? 1 : p.gear;
  if (!left && t >= duration() - 1e-10) {ax = ay = 0;}
  const double a = speed > 1e-9 ? gear * (vx * ax + vy * ay) / speed :
    ax * std::cos(yaw) + ay * std::sin(yaw);
  return {0, speed, gear * speed, a,
    {derivative(p.x, u, 0, p.duration), derivative(p.y, u, 0, p.duration), yaw},
    vx, vy, ax, ay, gear * speed * k, gear, k};
}
std::array<double, 2> QuinticTrajectory::jerk(double t) const
{
  auto [i, u] = locate(pieces_, t, false); const auto & p = pieces_[i];
  return {derivative(p.x, u, 3, p.duration), derivative(p.y, u, 3, p.duration)};
}
}  // namespace car_swarm_agent
