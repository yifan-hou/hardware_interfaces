#ifndef _DELTA_DJI_CLASS_HEADER_
#define _DELTA_DJI_CLASS_HEADER_

#include "hardware_interfaces/delta_interfaces.h"

#include "external/robomaster_hardware/include/Robomaster/robomaster_communicator.h"

class DeltaDJI: public DeltaInterfaces {
public:
  DeltaDJI();
  ~DeltaDJI();
  
};

#endif
