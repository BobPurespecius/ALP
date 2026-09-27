#pragma once

#include <traj_utils/PolyTraj.h>
#include <cstdint>
#include <cstring>

namespace traj_utils {

inline std::uint64_t polyPayloadHash(const traj_utils::PolyTraj &message) {
  std::uint64_t hash = 1469598103934665603ULL;
  const auto mix_word = [&hash](const std::uint64_t word) {
    hash ^= word;
    hash *= 1099511628211ULL;
  };
  const auto mix_double = [&mix_word](const double value) {
    std::uint64_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    mix_word(bits);
  };
  mix_word(static_cast<std::uint64_t>(message.drone_id));
  mix_word(static_cast<std::uint64_t>(message.traj_id));
  mix_word(message.start_time.toNSec());
  mix_word(static_cast<std::uint64_t>(message.order));
  for (const double value : message.duration) mix_double(value);
  for (const double value : message.coef_x) mix_double(value);
  for (const double value : message.coef_y) mix_double(value);
  for (const double value : message.coef_z) mix_double(value);
  return hash == 0 ? 1 : hash;
}

}  // namespace traj_utils
