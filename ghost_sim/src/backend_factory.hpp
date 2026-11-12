#pragma once
#include <memory>
#include <string>

#include "gsim_core/sim_backend.hpp"

namespace gsim {

// gsim_backend <name> → the simulator of that name if it was built in (GSIM_WITH_*);
// anything else → nullptr with *error naming the simulators that were built.
std::unique_ptr<ISimBackend> makeBackend(const std::string& name, std::string* error);

}  // namespace gsim
