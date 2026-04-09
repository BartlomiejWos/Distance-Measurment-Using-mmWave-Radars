#pragma once

#include <pybind11/pybind11.h>
#include "ti-external/rf_api.h"
#include "ti-external/dca_types.h"

#define DCA1000_CAPTURE_ONGOING 1
#define DCA1000_CAPTURE_DONE    2
#define DCA1000_CAPTURE_STOP    3

namespace py = pybind11;

extern volatile char dcaCaptureMode;
extern py::function py_event_callback;

void EthernetEventHandlercallback(UINT16 u16CmdCode, UINT16 u16Status);
void DecodeCommandStatus(SINT32 s32Status, const SINT8 *strCommand);
