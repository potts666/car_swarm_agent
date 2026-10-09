#ifndef CAR_SWARM_AGENT_DEMO_VALIDATION_HPP
#define CAR_SWARM_AGENT_DEMO_VALIDATION_HPP

#include "car_swarm_agent/trajectory_yield.hpp"

namespace car_swarm_agent {

enum class AfterPlaybackDecision { Blocked, DynamicOnly, StaticAndDynamicVerified };

inline AfterPlaybackDecision assessAfterPlayback(const STResult & result)
{
  if (result.status != STStatus::Success || !result.trajectory ||
    !result.continuous || !result.continuous_verification ||
    result.continuous_verification->status != ContinuousCheckStatus::Safe)
  {
    return AfterPlaybackDecision::Blocked;
  }
  return result.continuous_verification->static_map_checked ?
    AfterPlaybackDecision::StaticAndDynamicVerified : AfterPlaybackDecision::DynamicOnly;
}

inline const char * afterPlaybackMessage(AfterPlaybackDecision decision)
{
  switch (decision) {
    case AfterPlaybackDecision::StaticAndDynamicVerified:
      return "静态地图、邻车模型及运动限制复核通过 (planned trajectory verified)";
    case AfterPlaybackDecision::DynamicOnly:
      return "静态碰撞未验证 (static collision NOT verified); 邻车模型及运动限制复核通过";
    default:
      return "连续复核未通过，禁止 AFTER 回放 (AFTER blocked)";
  }
}

}  // namespace car_swarm_agent
#endif
