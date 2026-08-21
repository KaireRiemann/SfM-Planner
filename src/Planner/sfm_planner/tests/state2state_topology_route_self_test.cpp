#include <general_core/state2state/state2state_topology_route.hpp>

#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {

void expect(const bool condition, const char *message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

bool near(const double lhs, const double rhs, const double tolerance = 1.0e-6) {
    return std::abs(lhs - rhs) <= tolerance;
}

}  // namespace

int main() {
    using general_planner::state2state_task::State2StateTopologyRouteRuntime;
    using general_planner::state2state_task::appendVerifiedBreadcrumb;
    using general_planner::state2state_task::buildRouteArcLength;
    using general_planner::state2state_task::projectRouteMonotonically;
    using general_planner::state2state_task::resetVerifiedBreadcrumb;
    using general_planner::state2state_task::shouldAttemptTopologyLocalRepair;
    using general_planner::state2state_task::sliceRouteByArcLength;
    using general_utils::Vec3f;
    using general_utils::vec_Vec3f;

    vec_Vec3f route{
        Vec3f(0.0, 0.0, 0.0),
        Vec3f(2.0, 0.0, 0.0),
        Vec3f(2.0, 3.0, 0.0)};
    std::vector<double> arc_length;
    buildRouteArcLength(route, arc_length);
    expect(arc_length.size() == 3, "arc length size");
    expect(near(arc_length.back(), 5.0), "arc length value");

    double projected_s = -1.0;
    Vec3f projected_point = Vec3f::Zero();
    expect(projectRouteMonotonically(route, arc_length, Vec3f(1.2, 0.4, 0.0),
                                     0.0, projected_s, projected_point),
           "initial projection");
    expect(near(projected_s, 1.2), "initial projection arc");
    expect(near(projected_point.x(), 1.2), "initial projection point");

    expect(projectRouteMonotonically(route, arc_length, Vec3f(0.1, 0.0, 0.0),
                                     2.5, projected_s, projected_point),
           "monotonic projection");
    expect(near(projected_s, 2.5), "monotonic lower bound");
    expect(near(projected_point.x(), 2.0) && near(projected_point.y(), 0.5),
           "monotonic projection point");

    vec_Vec3f prefix;
    expect(sliceRouteByArcLength(route, arc_length, 0.5, 3.5, prefix),
           "route slice");
    expect(prefix.size() == 3, "route slice points");
    expect(near(prefix.front().x(), 0.5), "route slice start");
    expect(near(prefix.back().x(), 2.0) && near(prefix.back().y(), 1.5),
           "route slice end");

    State2StateTopologyRouteRuntime runtime;
    expect(!runtime.policy_enabled.load(), "local-only default");
    runtime.setPolicy(true);
    expect(runtime.policy_enabled.load() && runtime.policy_generation.load() == 1,
           "enable topology policy");
    runtime.setPolicy(true);
    expect(runtime.policy_generation.load() == 1, "idempotent enable");
    runtime.setPolicy(false);
    expect(!runtime.policy_enabled.load() && runtime.policy_generation.load() == 2,
           "disable topology policy");

    // A local-map boundary is not an obstacle and must not bypass the bounded
    // rejoin attempt. The rejoin target remains on the complete return route.
    expect(shouldAttemptTopologyLocalRepair(
                   false, true,
                   general_planner::state2state_task::GlobalRouteSource::TOPOLOGY),
           "boundary-limited topology route needs local repair");
    expect(shouldAttemptTopologyLocalRepair(
                   true, false,
                   general_planner::state2state_task::GlobalRouteSource::TOPOLOGY),
           "blocked topology route needs local repair");
    expect(shouldAttemptTopologyLocalRepair(
                   false, false,
                   general_planner::state2state_task::GlobalRouteSource::BREADCRUMB),
           "breadcrumb route needs local repair");
    expect(shouldAttemptTopologyLocalRepair(
                   false, false,
                   general_planner::state2state_task::GlobalRouteSource::EXECUTED_HISTORY),
           "executed-history route needs local repair");
    expect(!shouldAttemptTopologyLocalRepair(
                    false, false,
                    general_planner::state2state_task::GlobalRouteSource::TOPOLOGY),
           "healthy topology route needs no local repair");

    resetVerifiedBreadcrumb(runtime.breadcrumb, Vec3f(0.0, 0.0, 1.5));
    expect(runtime.breadcrumb.active && runtime.breadcrumb.path.size() == 1,
           "breadcrumb home anchor");
    expect(runtime.breadcrumb.last_record_failure == "NONE",
           "breadcrumb recorder starts without a stale failure");
    runtime.breadcrumb.last_record_failure = "BREADCRUMB_SEGMENT_NOT_KNOWN_FREE";
    resetVerifiedBreadcrumb(runtime.breadcrumb, Vec3f(0.0, 0.0, 1.5));
    expect(runtime.breadcrumb.last_record_failure == "NONE",
           "new mission clears prior breadcrumb recorder failure");
    expect(appendVerifiedBreadcrumb(runtime.breadcrumb, Vec3f(1.0, 0.0, 1.5)),
           "breadcrumb append first segment");
    expect(appendVerifiedBreadcrumb(runtime.breadcrumb, Vec3f(1.0, 2.0, 1.5)),
           "breadcrumb append second segment");
    expect(runtime.breadcrumb.path.size() == 3 &&
               near(runtime.breadcrumb.arc_length.back(), 3.0),
           "breadcrumb accumulated length");

    std::cout << "state2state_topology_route_self_test passed\n";
    return 0;
}
