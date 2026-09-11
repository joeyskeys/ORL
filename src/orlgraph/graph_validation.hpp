#pragma once

#include "graph_ir.hpp"
#include "graph_schedule.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace orlgraph
{

enum class DiagnosticSeverity : std::uint8_t {
    Error,
    Warning,
    Note,
};

struct Diagnostic {
    DiagnosticSeverity severity = DiagnosticSeverity::Error;
    std::string code;
    std::string message;
    StableId node;
    StableId port;
    Provenance provenance;
};

struct ValidationResult {
    std::vector<Diagnostic> diagnostics;
    ScheduleResult schedule;

    bool ok() const;
    void error(std::string code, std::string message,
        StableId node = {}, StableId port = {});
};

ValidationResult validate(const GraphModule& module, const NodeRegistry& registry);

} // namespace orlgraph
