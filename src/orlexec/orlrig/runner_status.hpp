#pragma once

#include <string>
#include <vector>

namespace orlrig
{

struct RunnerStatus {
    bool ok = false;
    std::vector<std::string> errors;

    explicit operator bool() const { return ok; }
};

} // namespace orlrig
