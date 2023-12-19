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


/** @brief Build the factor Graph with initial values and a first set of measurements
 *
 * @param Graph reference to the factor graph object
 * @param Measurements reference to the dataset that contains the pseudo range measurements
 * @param Config reference to the factor graph config
 * @param Options solver option to estimate initial values
 * @param TimestampFirst first timestamp in the Dataset
 * @return nothing
 *
 */
void InitGraph(libRSF::FactorGraph &Graph,
               libRSF::SensorDataSet &Measurements,
               libRSF::FactorGraphConfig const &Config,
               ceres::Solver::Options Options,
               double TimestampFirst)
{
  /** build simple graph */
  libRSF::FactorGraphConfig SimpleConfig = Config;
  SimpleConfig.GNSS.ErrorModel.Type = libRSF::ErrorModelType::Gaussian;
  libRSF::FactorGraph SimpleGraph;

  SimpleGraph.addState(POSITION_STATE, libRSF::DataType::Point2, TimestampFirst);
  SimpleGraph.addState(CLOCK_ERROR_STATE, libRSF::DataType::ClockError, TimestampFirst);
  AddPseudorangeMeasurements(SimpleGraph, Measurements, SimpleConfig, TimestampFirst);

  /** solve */
  SimpleGraph.solve(Options);

  /**add first state variables */
  Graph.addState(POSITION_STATE, libRSF::DataType::Point2, TimestampFirst);
  Graph.addState(CLOCK_ERROR_STATE, libRSF::DataType::ClockError, TimestampFirst);
  Graph.addState(CLOCK_DRIFT_STATE, libRSF::DataType::ClockDrift, TimestampFirst);

  /** copy values to real graph */
  Graph.getStateData().getElement(POSITION_STATE, TimestampFirst).setMean(SimpleGraph.getStateData().getElement(POSITION_STATE, TimestampFirst).getMean());
  Graph.getStateData().getElement(CLOCK_ERROR_STATE, TimestampFirst).setMean(SimpleGraph.getStateData().getElement(CLOCK_ERROR_STATE, TimestampFirst).getMean());


  /** add first set of measurements */
  AddPseudorangeMeasurements(Graph, Measurements, Config, TimestampFirst);
}

/** @brief Adds a pseudorange measurement to the graph
 *
 * @param Graph reference to the factor graph object
 * @param Measurements reference to the dataset that contains the measurement
 * @param Config reference to the factor graph config object that specifies the motion model
 * @param Timestamp a double timestamp of the current position
 * @return nothing
 *
 */
void AddPseudorangeMeasurements(libRSF::FactorGraph &Graph,
                                libRSF::SensorDataSet &Measurements,
                                libRSF::FactorGraphConfig const &Config,
                                double Timestamp)
{
  libRSF::StateList ListPseudorange;
  libRSF::Data Pseudorange;
  libRSF::GaussianDiagonal<1> NoisePseudorange;

  ListPseudorange.add(POSITION_STATE, Timestamp);
  ListPseudorange.add(CLOCK_ERROR_STATE, Timestamp);

  int SatNumber = Measurements.countElement(libRSF::DataType::Pseudorange2, Timestamp);

  for(int SatCounter = 0; SatCounter < SatNumber; ++SatCounter)
  {
    /** get measurement */
    Pseudorange = Measurements.getElement(libRSF::DataType::Pseudorange2, Timestamp, SatCounter);

    /** default error model */
    static libRSF::GaussianMixture<1> GMM((libRSF::Vector2() << 0, 0).finished(),
                                           (libRSF::Vector2() << 10, 100).finished(),
                                           (libRSF::Vector2() << 0.5, 0.5).finished());

    /** add factor */
    switch(Config.GNSS.ErrorModel.Type)
    {
      case libRSF::ErrorModelType::Gaussian:
        NoisePseudorange.setStdDevDiagonal(Pseudorange.getStdDevDiagonal());
        Graph.addFactor<libRSF::FactorType::Pseudorange2>(ListPseudorange, Pseudorange, NoisePseudorange);
        break;

      case libRSF::ErrorModelType::DCS:
        NoisePseudorange.setStdDevDiagonal(Pseudorange.getStdDevDiagonal());
        Graph.addFactor<libRSF::FactorType::Pseudorange2>(ListPseudorange, Pseudorange, NoisePseudorange, new libRSF::DCSLoss(1.0));
        break;

      case libRSF::ErrorModelType::cDCE:
        NoisePseudorange.setStdDevDiagonal(libRSF::Vector1::Ones());
        Graph.addFactor<libRSF::FactorType::Pseudorange2>(ListPseudorange, Pseudorange, NoisePseudorange, new libRSF::cDCELoss(Pseudorange.getStdDevDiagonal()[0]));
        break;

      case libRSF::ErrorModelType::GMM:
        if (Config.GNSS.ErrorModel.GMM.MixtureType == libRSF::ErrorModelMixtureType::MaxMix)
        {
          static libRSF::MaxMix1 NoisePseudorangeMaxMix(GMM);
          Graph.addFactor<libRSF::FactorType::Pseudorange2>(ListPseudorange, Pseudorange, NoisePseudorangeMaxMix);
        }
        else if (Config.GNSS.ErrorModel.GMM.MixtureType == libRSF::ErrorModelMixtureType::SumMix)
        {
          static libRSF::SumMix1 NoisePseudorangeSumMix(GMM);
          Graph.addFactor<libRSF::FactorType::Pseudorange2>(ListPseudorange, Pseudorange, NoisePseudorangeSumMix);
        }
        else
        {
          PRINT_ERROR("Wrong error model mixture type!");
        }

        break;

      default:
        PRINT_ERROR("Wrong error model type: ", Config.GNSS.ErrorModel.Type);
        break;
    }
  }
}

/** @brief Use a GMM to estimate the error distribution of a factor
*
* @param Graph reference to the factor graph object
* @param Config reference to the factor graph config object that specifies the motion model
* @param NumberOfComponents how many Gaussian components should be used
* @param Timestamp current timestamp
* @return nothing
*
*/
void TuneErrorModel(libRSF::FactorGraph &Graph,
                    libRSF::FactorGraphConfig &Config,
                    int NumberOfComponents)
{
  libRSF::GaussianMixture<1> GMM;
  if(Config.GNSS.ErrorModel.GMM.TuningType == libRSF::ErrorModelTuningType::EM)
  {
    /** fill empty GMM */
    if(GMM.getNumberOfComponents() == 0)
    {
      libRSF::GaussianComponent<1> Component;

      for(int nComponent = 0; nComponent < NumberOfComponents; ++nComponent)
      {

        Component.setParamsStdDev((libRSF::Vector1() << pow(10, nComponent+1)).finished(),
                                  (libRSF::Vector1() << 0.0).finished(),
                                  (libRSF::Vector1() << 1.0/NumberOfComponents).finished());

        GMM.addComponent(Component);
      }
    }

    libRSF::Matrix ErrorData;
    Graph.computeUnweightedErrorMatrix(libRSF::FactorType::Pseudorange2, ErrorData);

    /** call the EM algorithm */
    libRSF::GaussianMixture<1>::EstimationConfig GMMConfig;
    GMMConfig.EstimationAlgorithm = libRSF::ErrorModelTuningType::EM;
    GMM.estimate(ErrorData, GMMConfig);

    /** remove offset of the first "LOS" component */
    GMM.removeOffsetLegacy();

    /** apply error model */
    if(Config.GNSS.ErrorModel.GMM.MixtureType == libRSF::ErrorModelMixtureType::SumMix)
    {
      libRSF::SumMix1 NewSMModel(GMM);
      Graph.setNewErrorModel(libRSF::FactorType::Pseudorange2, NewSMModel);
    }
    else if(Config.GNSS.ErrorModel.GMM.MixtureType == libRSF::ErrorModelMixtureType::MaxMix)
    {
      libRSF::MaxMix1 NewMMModel(GMM);
      Graph.setNewErrorModel(libRSF::FactorType::Pseudorange2, NewMMModel);
    }
  }
}

bool ParseErrorModel(const std::string &ErrorModel, libRSF::FactorGraphConfig &Config)
{
  if(ErrorModel == "gauss")
  {
    Config.GNSS.ErrorModel.Type = libRSF::ErrorModelType::Gaussian;
    Config.GNSS.ErrorModel.GMM.TuningType = libRSF::ErrorModelTuningType::None;
  }
  else if(ErrorModel == "dcs")
  {
    Config.GNSS.ErrorModel.Type = libRSF::ErrorModelType::DCS;
    Config.GNSS.ErrorModel.GMM.TuningType = libRSF::ErrorModelTuningType::None;
  }
  else if(ErrorModel == "cdce")
  {
    Config.GNSS.ErrorModel.Type = libRSF::ErrorModelType::cDCE;
    Config.GNSS.ErrorModel.GMM.TuningType = libRSF::ErrorModelTuningType::None;
  }
  else if(ErrorModel == "sm")
  {
    Config.GNSS.ErrorModel.Type = libRSF::ErrorModelType::GMM;
    Config.GNSS.ErrorModel.GMM.MixtureType = libRSF::ErrorModelMixtureType::SumMix;
    Config.GNSS.ErrorModel.GMM.TuningType = libRSF::ErrorModelTuningType::None;
  }
  else if(ErrorModel == "mm")
  {
    Config.GNSS.ErrorModel.Type = libRSF::ErrorModelType::GMM;
    Config.GNSS.ErrorModel.GMM.MixtureType = libRSF::ErrorModelMixtureType::MaxMix;
    Config.GNSS.ErrorModel.GMM.TuningType = libRSF::ErrorModelTuningType::None;
  }
  else if(ErrorModel == "stsm")
  {
    Config.GNSS.ErrorModel.Type = libRSF::ErrorModelType::GMM;
    Config.GNSS.ErrorModel.GMM.MixtureType = libRSF::ErrorModelMixtureType::SumMix;
    Config.GNSS.ErrorModel.GMM.TuningType = libRSF::ErrorModelTuningType::EM;
  }
  else if(ErrorModel == "stmm")
  {
    Config.GNSS.ErrorModel.Type = libRSF::ErrorModelType::GMM;
    Config.GNSS.ErrorModel.GMM.MixtureType = libRSF::ErrorModelMixtureType::MaxMix;
    Config.GNSS.ErrorModel.GMM.TuningType = libRSF::ErrorModelTuningType::EM;
  }
  else
  {
    PRINT_ERROR("Wrong Error Model: ", ErrorModel);
    return false;
  }

  return true;
}

bool CreateGraphAndSolve(const std::string& DataSet,
                const std::string& GroundTruth,
                const std::string& Output,
                libRSF::StateDataSet &Result,
                libRSF::SensorDataSet &GT,
                const std::string& ErrorModel)
{
  libRSF::FactorGraphConfig Config;
  /** read filenames */
  Config.InputFile = std::string(DataSet);
  Config.OutputFile = std::string(Output);

  /** parse the error model string */
  if (!ParseErrorModel(std::string(ErrorModel), Config))
  {
    return 1;
  }

  ceres::Solver::Options SolverOptions;
  SolverOptions.minimizer_progress_to_stdout = false;
  SolverOptions.use_nonmonotonic_steps = true;
  SolverOptions.max_num_iterations = 1000;
  SolverOptions.trust_region_strategy_type = ceres::TrustRegionStrategyType::DOGLEG;
  SolverOptions.dogleg_type = ceres::DoglegType::SUBSPACE_DOGLEG;
  SolverOptions.num_threads = static_cast<int>(std::thread::hardware_concurrency());
  // SolverOptions.max_solver_time_in_seconds = 1.0;

  const int NumberOfComponents = 8;

  /** read input data */
  libRSF::SensorDataSet RangeMeasurements;
  libRSF::ReadDataFromFile(std::string(DataSet), RangeMeasurements);
  /** load ground truth */
  libRSF::ReadDataFromFile(std::string(GroundTruth), GT);

  /** Build optimization problem from sensor data */
  libRSF::FactorGraph Graph;

  /** find first time stamp */
  double Time, TimeFirst = 0.0, TimeOld = 0.0;
  int nTimestamp = 0;
  if (!RangeMeasurements.getTimeFirstOverall(TimeFirst))
  {
    PRINT_ERROR("Dataset is empty!");
    return 1;
  }
   /** loop over timestamps */
  RangeMeasurements.getTimeFirst(PSEUDORANGE_MEASUREMENT, TimeFirst);
  Time = TimeFirst;
  TimeOld = TimeFirst;

  /** add fist variables and factors */
  InitGraph(Graph, RangeMeasurements, Config, SolverOptions, TimeFirst);

  /** solve multiple times with refined model to achieve good initial convergence */
  Graph.solve(SolverOptions);
  TuneErrorModel(Graph, Config, NumberOfComponents);
  Graph.solve(SolverOptions);

  /** safe result at first timestamp */
  Result.addElement(POSITION_STATE, Graph.getStateData().getElement(POSITION_STATE, Time, 0));

  /** clock error drift (CCED) model noise properties */
  libRSF::Vector1 StdDev, StdDevCCE;
  StdDev << STDDEV_RANGE;
  StdDevCCE << STDDEV_CCE;
  libRSF::GaussianDiagonal<1> NoiseModelRange, NoiseModelCCE;
  NoiseModelRange.setStdDevDiagonal(StdDev);
  NoiseModelCCE.setStdDevDiagonal(StdDevCCE);

  /** hard coded constant clock error drift (CCED) model noise properties */
  libRSF::Vector2 StdCCED;
  StdCCED << STD_CCED;
  libRSF::GaussianDiagonal<2> NoiseCCED;
  NoiseCCED.setStdDevDiagonal(StdCCED);

  while(RangeMeasurements.getTimeNext(PSEUDORANGE_MEASUREMENT, Time, Time)) {

    /** add position variables to graph */
    Graph.addState(POSITION_STATE, libRSF::DataType::Point2, Time);
    // /** add offset variables to graph */
    // Graph.addState(OFFSET_STATE, libRSF::DataType::ClockError, Time);
    Graph.addState(CLOCK_ERROR_STATE, libRSF::DataType::ClockError, Time);
    Graph.addState(CLOCK_DRIFT_STATE, libRSF::DataType::ClockDrift, Time);

    
    libRSF::StateList RangeList, ConstValList;
    RangeList.add(POSITION_STATE, TimeOld);
    RangeList.add(POSITION_STATE, Time);
    
    // /** add constant value model*/
    // if (Time > TimeFirst)
    // {
    //   ConstValList.add(OFFSET_STATE, TimeOld);
    //   ConstValList.add(OFFSET_STATE, Time);
    //   Graph.addFactor<libRSF::FactorType::ConstVal1>(ConstValList, NoiseModelCCE);
    //   ConstValList.clear();
    // }

    /** add clock drift model */
    libRSF::StateList ClockList;
    ClockList.add(CLOCK_ERROR_STATE, TimeOld);
    ClockList.add(CLOCK_DRIFT_STATE, TimeOld);
    ClockList.add(CLOCK_ERROR_STATE, Time);
    ClockList.add(CLOCK_DRIFT_STATE, Time);
    Graph.addFactor<libRSF::FactorType::ConstDrift1>(ClockList, NoiseCCED);

    /** add all pseudo range measurements of with current timestamp */
    AddPseudorangeMeasurements(Graph, RangeMeasurements, Config, Time);

    // /** loop over measurements */
    // for (int nMeasurement = RangeMeasurements.countElement(PSEUDORANGE_MEASUREMENT, Time); nMeasurement > 0; nMeasurement--)
    // {
    //   Graph.addFactor<libRSF::FactorType::Pseudorange2>(RangeList, RangeMeasurements.getElement(PSEUDORANGE_MEASUREMENT, Time, nMeasurement-1), NoiseModelRange);
    // }

    /** tune self-tuning error model */
    TuneErrorModel(Graph, Config, NumberOfComponents);
    /** solve graph */
    Graph.solve(SolverOptions);
    Graph.printReport();

    /** save data after optimization */
    Result.addElement(POSITION_STATE, Graph.getStateData().getElement(POSITION_STATE, Time, 0));

    /** apply sliding window */
    Graph.removeAllFactorsOutsideWindow(10, Time);
    Graph.removeAllStatesOutsideWindow(10, Time);

    RangeList.clear();

    TimeOld = Time;
    
    nTimestamp++;
  }

  // /** solve graph */
  // Graph.solve(SolverOptions);

  Graph.printReport();

  return 0;

}

TEST(Test_desa_Example_UWB_Pseudorange_optimized, 230905_pseudoranges_optimized){
  /** calculate example */
  libRSF::StateDataSet Result;
  libRSF::SensorDataSet GT;
  ASSERT_FALSE(CreateGraphAndSolve(DATA_UWB_PR_IN, DATA_GT_UWB, DATA_UWB_PR_OUT, Result, GT, "stmm")) << "Error calculating example";

  /** calculate RMSE */
  const double ATE = libRSF::ATE(libRSF::DataType::Point2, GT, POSITION_STATE, Result);
  /** export estimates to file */
  libRSF::WriteDataToFile(DATA_UWB_PR_OUT, POSITION_STATE, Result, false);
  libRSF::WriteDataToFile(DATA_UWB_PR_OUT, OFFSET_STATE, Result, true);
  PRINT_LOGGING("ATE: ", ATE, "m");
  // EXPECT_NEAR(ATE, 0.23, 0.01);
}