#ifndef DESA_EXAMPLE_UWB_PSEUDORANGE_H_INCLUDED
#define DESA_EXAMPLE_UWB_PSEUDORANGE_H_INCLUDED

#include "libRSF.h"

/** use define to prevent typos*/
#define POSITION_STATE "Position"
#define OFFSET_STATE "Offset"
#define PSEUDORANGE_MEASUREMENT libRSF::DataType::Pseudorange2

#define STDDEV_RANGE  0.1
#define STDDEV_CCE    1.0
#define OFFSET        42.0

void CreateData (libRSF::SensorDataSet &RangeMeasurements);

#endif // DESA_EXAMPLE_UWB_PSEUDORANGE_H_INCLUDED