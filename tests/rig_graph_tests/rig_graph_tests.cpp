#if __has_include(<catch2/catch_all.hpp>)

#include <catch2/catch_all.hpp>

#include "orlrig/graph_resources.hpp"

using namespace orlrig;
using namespace orlgraph;

TEST_CASE("standard LBS rig graph exposes resources and dependencies",
    "[orlrig][graph]")
{
    const RigGraph graph = make_lbs_graph();
    const auto validation = graph.validate();
    REQUIRE(validation.ok());
    REQUIRE(validation.schedule.order.size() == 2);
    REQUIRE(validation.schedule.order[0] == StableId{"capture_bind"});
    REQUIRE(validation.schedule.order[1] == StableId{"deform"});

    REQUIRE(graph.module.resource(graph.resources.joints) != nullptr);
    REQUIRE(graph.module.resource(graph.resources.weights) != nullptr);
    REQUIRE(graph.module.resource(graph.resources.posed_positions) != nullptr);
    REQUIRE(graph.module.outputs().contains(graph.resources.posed_positions));
}

#endif
