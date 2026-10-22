#include "selected_adapter.hpp"

int main() {
    return ORL::Rhi::SelectedAdapter::backend_name().empty() ? 1 : 0;
}
