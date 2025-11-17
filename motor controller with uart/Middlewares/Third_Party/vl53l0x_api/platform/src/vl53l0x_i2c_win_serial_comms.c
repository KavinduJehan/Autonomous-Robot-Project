#include "vl53l0x_i2c_platform.h"

/*
 * This file previously contained a Windows-specific UART bridge.
 * The embedded firmware does not rely on these helpers, so we keep
 * the translation unit empty to satisfy the build system without
 * dragging in desktop dependencies.
 */
