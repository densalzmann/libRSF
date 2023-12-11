   /**
* @file desa_Example_UWB_Pseudorange.cpp
* @author Dennis Salzmann
* @date 11.12.2023
* @brief An example application to check UWB measurements for libRSF suite as pseudorange
* @copyright GNU Public License.
*
*/

#include "Test_desa_Example_UWB_Pseudorange.h"
#include "TestUtils.h"
#include "gtest/gtest.h"

/** define paths */
#define CONFIG "config/Default_Ranging.yaml"
#define DATA_UWB_PR "datasets/Ranging_UWB/230905_pseudoranges.txt"
#define DATA_GT_UWB "datasets/Ranging_UWB/230905_GT.txt"

#ifndef TESTMODE // only compile main if not used in test context

bool RunRanging(const std::string& DataSet,
                const std::string& GroundTruth,
               libRSF::StateDataSet &Result,
               libRSF::SensorDataSet &GT)
{
  
  /** read input data */
  libRSF::SensorDataSet RangeMeasurements;
  libRSF::ReadDataFromFile(std::string(DataSet), RangeMeasurements);
  /** load ground truth */
  // libRSF::ReadDataFromFile(std::string(GroundTruth), GT);

  /** find first time stamp */
  double Time, TimeFirst = 0.0, TimeOld = 0.0;

  libRSF::Vector1 StdDev, StdDevCCE;
  StdDev << STDDEV_RANGE;
  StdDevCCE << STDDEV_CCE;

  ceres::Solver::Options SolverOptions;
  SolverOptions.trust_region_strategy_type = ceres::TrustRegionStrategyType::DOGLEG;
  SolverOptions.dogleg_type = ceres::DoglegType::SUBSPACE_DOGLEG;
  SolverOptions.num_threads = static_cast<int>(std::thread::hardware_concurrency());
  SolverOptions.minimizer_progress_to_stdout = true;

  /** Build optimization problem from sensor data */
  libRSF::FactorGraph Graph;
  libRSF::StateList RangeList, ConstValList;
  libRSF::GaussianDiagonal<1> NoiseModelRange, NoiseModelCCE;
  NoiseModelRange.setStdDevDiagonal(StdDev);
  NoiseModelCCE.setStdDevDiagonal(StdDevCCE);
   /** loop over timestamps */
  RangeMeasurements.getTimeFirst(PSEUDORANGE_MEASUREMENT, TimeFirst);
  Time = TimeFirst;
  do {
    /** add position variables to graph */
    SimpleGraph.addState(POSITION_STATE, libRSF::DataType::Point2, Time);

    /** add offset variables to graph */
    SimpleGraph.addState(OFFSET_STATE, libRSF::DataType::ClockError, Time);

    RangeList.add(POSITION_STATE, Time);
    RangeList.add(OFFSET_STATE, Time);

    /** add constant value model*/
    if (Time > TimeFirst)
    {
      ConstValList.add(OFFSET_STATE, TimeOld);
      ConstValList.add(OFFSET_STATE, Time);
      SimpleGraph.addFactor<libRSF::FactorType::ConstVal1>(ConstValList, NoiseModelCCE);
      ConstValList.clear();
    }

    /** loop over measurements */
    for (int nMeasurement = RangeMeasurements.countElement(PSEUDORANGE_MEASUREMENT, Time); nMeasurement > 0; nMeasurement--)
    {
      SimpleGraph.addFactor<libRSF::FactorType::Pseudorange2>(RangeList, RangeMeasurements.getElement(PSEUDORANGE_MEASUREMENT, Time, nMeasurement-1), NoiseModelRange);
    }
    RangeList.clear();

    TimeOld = Time;
  }
  while(RangeMeasurements.getTimeNext(PSEUDORANGE_MEASUREMENT, Time, Time));

  /** solve graph */
  SimpleGraph.solve(SolverOptions);
  SimpleGraph.printReport();

  return 0;

}

TEST(App_Ranging_2D, M3500_heavy_tailed){
  /** calculate example */
  libRSF::StateDataSet Result;
  libRSF::SensorDataSet GT;
  ASSERT_FALSE(RunRanging(DATA_UWB_PR, DATA_GT_UWB, Result, GT)) << "Error calculating example";

  /** calculate RMSE */
  const double ATE = libRSF::ATE(libRSF::DataType::Point2, GT, POSITION_STATE, Result);
  PRINT_LOGGING("ATE: ", ATE, "m");
//   EXPECT_NEAR(ATE, 0.23, 0.01);
}