#include "policy_watchdog.h"
#include <cassert>
#include <iostream>
#include <limits>

int main() {
  using pnp7_policy::fresh;
  assert(!fresh(0, 0, 1000000));
  assert(!fresh(100, 101, 110));  // queued command from previous controller run
  assert(!fresh(200, 100, 199));  // future monotonic timestamp
  assert(fresh(200, 100, 249999999 + 200));
  assert(!fresh(200, 100, 250000000 + 200));
  Eigen::Vector3d p(.5, .2, .5);
  Eigen::Quaterniond r = Eigen::Quaterniond::Identity();
  assert(pnp7_policy::valid(p, r, .05, "fr3_link0"));
  assert(!pnp7_policy::valid(p, r, .101, "fr3_link0"));
  assert(!pnp7_policy::valid(p, r, -.051, "fr3_link0"));
  assert(!pnp7_policy::valid(p, r, 0, "panda_link0"));
  auto bad_r = r;
  bad_r.coeffs().setZero();
  assert(!pnp7_policy::valid(p, bad_r, 0, "fr3_link0"));
  auto bad_p = p;
  bad_p[0] = std::numeric_limits<double>::quiet_NaN();
  assert(!pnp7_policy::valid(bad_p, r, 0, "fr3_link0"));
  assert(!pnp7_policy::valid(p, r, std::numeric_limits<double>::quiet_NaN(), "fr3_link0"));
  assert(pnp7_policy::close(p + Eigen::Vector3d(.001, 0, 0), r, p, r));
  assert(pnp7_policy::close(p + Eigen::Vector3d(.0899, 0, 0), r, p, r));
  assert(!pnp7_policy::close(p + Eigen::Vector3d(.0901, 0, 0), r, p, r));
  assert(pnp7_policy::close(p, Eigen::Quaterniond(Eigen::AngleAxisd(.5999, Eigen::Vector3d::UnitZ())), p, r));
  assert(!pnp7_policy::close(p, Eigen::Quaterniond(Eigen::AngleAxisd(.6001, Eigen::Vector3d::UnitZ())), p, r));
  std::cout << "POLICY_WATCHDOG_TEST_PASS\n";
}
