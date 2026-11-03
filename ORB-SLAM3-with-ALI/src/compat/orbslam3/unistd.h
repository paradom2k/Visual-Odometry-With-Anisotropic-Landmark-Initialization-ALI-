// Stand-in for <unistd.h> (route E, rebuilt repo).
//
// Three headers of the ported tree include it (`System.h`, `Config.h`/`Settings.h` and LocalMapping) and use
// nothing from it: the POSIX calls that matter on Windows (`usleep`) are mapped in compat/jo_compat.h, which
// is force-included into every translation unit of the target.
#pragma once

#include <cstddef>
#include <cstdint>

// Kept minimal on purpose: anything that needs a real POSIX call belongs in jo_compat.h so there is exactly
// one place to look when a POSIX habit of the ported tree has to be redirected.
