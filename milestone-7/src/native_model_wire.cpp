// SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
// Compile the exact portable C codec with C linkage in both host and QNX builds.
extern "C" {
#include "native_model_wire/protocol.c"
}
