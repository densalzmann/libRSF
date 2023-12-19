#ifndef DESA_EXAMPLE_UWB_PSEUDORANGE_H_INCLUDED
#define DESA_EXAMPLE_UWB_PSEUDORANGE_H_INCLUDED

#include "libRSF.h"

/** use define to prevent typos*/
#define POSITION_STATE "Position"
#define OFFSET_STATE "Offset"
#define CLOCK_ERROR_STATE "ClockError"
#define CLOCK_DRIFT_STATE "ClockDrift"
#define PSEUDORANGE_MEASUREMENT libRSF::DataType::Pseudorange2

#define STDDEV_RANGE  0.3
#define STDDEV_CCE    0.1
#define STD_CCED    0.05, 0.01

/** Build the factor Graph with initial values and a first set of measurements */
void InitGraph(libRSF::FactorGraph &Graph,
               libRSF::SensorDataSet &Measurements,
               libRSF::FactorGraphConfig const &Config,
               ceres::Solver::Options Options,
               double TimestampFirst);

/** Adds a pseudorange measurement to the graph */
void AddPseudorangeMeasurements(libRSF::FactorGraph& Graph,
                                libRSF::SensorDataSet & Measurements,
                                libRSF::FactorGraphConfig const &Config,
                                double Timestamp);

/** use EM algorithm to tune the gaussian mixture model */
void TuneErrorModel(libRSF::FactorGraph &Graph,
                    libRSF::FactorGraphConfig &Config,
                    int NumberOfComponents);

/** parse string from command line to select error model for GNSS*/
bool ParseErrorModel(const std::string &ErrorModel, libRSF::FactorGraphConfig &Config);

#endif // DESA_EXAMPLE_UWB_PSEUDORANGE_H_INCLUDED