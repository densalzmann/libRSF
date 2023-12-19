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
#define CURRENT_PATH "/home/reimagined-octo-fortnight/10_CoreFuncts/Infusion/Modules/30_CAMod/libRSF/"
#define CONFIG CURRENT_PATH "config/Default_Ranging.yaml"
#define DATA_UWB_PR_IN CURRENT_PATH "datasets/Ranging_UWB/230905_pseudoranges_real_timestamps_input.txt"
#define DATA_UWB_PR_OUT CURRENT_PATH "datasets/Ranging_UWB/230905_pseudoranges_output.txt"
#define DATA_GT_UWB CURRENT_PATH "datasets/Ranging_UWB/230905_GT.txt"


bool RunRanging(const std::string& DataSet,
                const std::string& GroundTruth,
                const std::string& Output,
               libRSF::StateDataSet &Result,
               libRSF::SensorDataSet &GT)
{
  libRSF::FactorGraphConfig Config;
  /** read filenames */
  Config.InputFile = std::string(DataSet);
  Config.OutputFile = std::string(Output);
  /** parse the error model string */
  Config.GNSS.ErrorModel.Type = libRSF::ErrorModelType::Gaussian;
  Config.GNSS.ErrorModel.GMM.TuningType = libRSF::ErrorModelTuningType::None;

  /** read input data */
  libRSF::SensorDataSet RangeMeasurements;
  libRSF::ReadDataFromFile(std::string(DataSet), RangeMeasurements);
  /** load ground truth */
  libRSF::ReadDataFromFile(std::string(GroundTruth), GT);

  /** find first time stamp */
  double Time, TimeFirst = 0.0, TimeOld = 0.0;
  int nTimestamp = 0;

  libRSF::Vector1 StdDev, StdDevCCE;
  StdDev << STDDEV_RANGE;
  StdDevCCE << STDDEV_CCE;

  ceres::Solver::Options SolverOptions;
  SolverOptions.max_num_iterations = 10000;
  SolverOptions.trust_region_strategy_type = ceres::TrustRegionStrategyType::DOGLEG;
  SolverOptions.dogleg_type = ceres::DoglegType::SUBSPACE_DOGLEG;
  SolverOptions.num_threads = static_cast<int>(std::thread::hardware_concurrency());
  SolverOptions.minimizer_progress_to_stdout = true;

  const int NumberOfComponents = 2;

  /** Build optimization problem from sensor data */
  libRSF::FactorGraph Graph;
  libRSF::StateList RangeList, ConstValList;
  libRSF::GaussianDiagonal<1> NoiseModelRange, NoiseModelCCE;
  NoiseModelRange.setStdDevDiagonal(StdDev);
  NoiseModelCCE.setStdDevDiagonal(StdDevCCE);
  if (!RangeMeasurements.getTimeFirstOverall(TimeFirst))
  {
    PRINT_ERROR("Dataset is empty!");
    return 1;
  }
   /** loop over timestamps */
  RangeMeasurements.getTimeFirst(PSEUDORANGE_MEASUREMENT, TimeFirst);
  Time = TimeFirst;

  do {
    /** add position variables to graph */
    Graph.addState(POSITION_STATE, libRSF::DataType::Point2, Time);

    /** add offset variables to graph */
    Graph.addState(OFFSET_STATE, libRSF::DataType::ClockError, Time);

    RangeList.add(POSITION_STATE, Time);
    RangeList.add(OFFSET_STATE, Time);

    /** add constant value model*/
    if (Time > TimeFirst)
    {
      ConstValList.add(OFFSET_STATE, TimeOld);
      ConstValList.add(OFFSET_STATE, Time);
      Graph.addFactor<libRSF::FactorType::ConstVal1>(ConstValList, NoiseModelCCE);
      ConstValList.clear();
    }

    /** loop over measurements */
    for (int nMeasurement = RangeMeasurements.countElement(PSEUDORANGE_MEASUREMENT, Time); nMeasurement > 0; nMeasurement--)
    {
      Graph.addFactor<libRSF::FactorType::Pseudorange2>(RangeList, RangeMeasurements.getElement(PSEUDORANGE_MEASUREMENT, Time, nMeasurement-1), NoiseModelRange);
    }

    RangeList.clear();

    TimeOld = Time;
    
    nTimestamp++;
  }
  while(RangeMeasurements.getTimeNext(PSEUDORANGE_MEASUREMENT, Time, Time));

  /** solve graph */
  Graph.solve(SolverOptions);

  Graph.printReport();
  PRINT_LOGGING("Elements in the dataset: ", RangeMeasurements.countElements(PSEUDORANGE_MEASUREMENT));

  Result = Graph.getStateData();

  return 0;

}

TEST(Test_desa_Example_UWB_Pseudorange, 230905_pseudoranges){
  /** calculate example */
  libRSF::StateDataSet Result;
  libRSF::SensorDataSet GT;
  ASSERT_FALSE(RunRanging(DATA_UWB_PR_IN, DATA_GT_UWB, DATA_UWB_PR_OUT, Result, GT)) << "Error calculating example";

  /** calculate RMSE */
  const double ATE = libRSF::ATE(libRSF::DataType::Point2, GT, POSITION_STATE, Result);
  /** export estimates to file */
  libRSF::WriteDataToFile(DATA_UWB_PR_OUT, POSITION_STATE, Result, false);
  libRSF::WriteDataToFile(DATA_UWB_PR_OUT, OFFSET_STATE, Result, true);
  PRINT_LOGGING("ATE: ", ATE, "m");
  // EXPECT_NEAR(ATE, 0.23, 0.01);
}