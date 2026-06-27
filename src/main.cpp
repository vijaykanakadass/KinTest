// kinplay M1 CLI: prints FK for the built-in planar 2R arm at a few joint
// configurations. The interactive viewer arrives in milestone M5 (CLAUDE.md §9).

#include <array>
#include <cmath>
#include <cstdio>

#include "core/mechanism.h"

namespace {
constexpr double kPi = 3.141592653589793238462643383279502884;
}

int main() {
  using namespace kp;

  const double L1 = 1.0;
  const double L2 = 1.0;
  Mechanism arm = make_planar_2r(L1, L2);
  const int ee = static_cast<int>(arm.links().size()) - 1;

  struct Sample { double q1, q2; };
  const std::array<Sample, 4> samples = {{
      {0.0,        0.0       },
      {kPi / 2.0,  0.0       },
      {0.0,        kPi / 2.0 },
      {kPi / 4.0,  kPi / 4.0 },
  }};

  std::printf("planar_2r  L1=%.3f  L2=%.3f  dof=%d\n", L1, L2, arm.dof());
  std::printf("%8s %8s | %12s %12s %12s\n",
              "q1", "q2", "ee.x", "ee.y", "ee.z");
  for (const auto& s : samples) {
    VecX q(2);
    q << s.q1, s.q2;
    const Transform T = arm.link_pose(q, ee);
    const Vec3 p = T.translation();
    std::printf("%8.4f %8.4f | %12.6f %12.6f %12.6f\n",
                s.q1, s.q2, p.x(), p.y(), p.z());
  }
  return 0;
}
