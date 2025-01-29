/*
 * Copyright (C) 2025: Arizona Board of Regents on Behalf of the University of Arizona
 */

 /**
  * @file RangeEstimator.h
  * @brief Apache Strap-Down Pilotage Render/Display RangeEstimator header file.
  *
 * @author ReliaSolve.
 * @date January 29, 2025.
 */

#pragma once
#include <string>
#include <memory>
#include <ImageStatistics.h>

namespace asdp {
  namespace render {

    /// @brief RangeEstimator pure virtual base class that defines the interface.
    class RangeEstimator {
    public:

      /// @brief Report the current value of the range in a thread-safe manner.
      /// @details Reports the current value of the range from 0-1.
      /// @param [out] minVal Filled in with the low end of the range on success, 0.0 on failure.
      /// @param [out] maxVal Filled in with the high end of the range on success, 1.0 on failure.
      /// @return Empty string on success, string describing the failure on failure.
      virtual std::string GetCurrentRange(double &minVal, double &maxVal) = 0;

      /// @brief Test for this class and derived classes that returns a string.
      /// @return A string that describes the test results, empty on succes and error message on failure.
      static std::string Test();
    };

    /// @brief Specialized RangeEstimator that has a constant range specified in the constructor.
    class RangeEstimatorFixed : public RangeEstimator {
    public:
      /// @brief Constructor
      /// @param numEntries The number of entries in the mapping.
      RangeEstimatorFixed(double minVal = 0, double maxVal = 1) : m_minVal(minVal), m_maxVal(maxVal) {}

      std::string GetCurrentRange(double& minVal, double& maxVal) override;

    protected:
      double m_minVal; ///< The low end of the range.
      double m_maxVal; ///< The high end of the range.
    };

    /// @brief Specialized RangeEstimator that uses a MeanStdGroup with specified scales of standard dev below
    /// and above the mean.
    class RangeEstimatorStdRanges : public RangeEstimator {
    public:
      /// @brief Constructor
      /// @param meanStdGroup The mean and standard deviation group to use for calculation.
      /// @param numStdBelow The number of standard deviations below the mean, clamped at the value 0.
      /// @param numStdAbove The number of standard deviations above the mean, clamped at the value 1.
      RangeEstimatorStdRanges(std::shared_ptr<asdp::render::imageStatistics::MeanStdGroup> meanStdGroup,
          double numStdBelow, double numStdAbove)
        : m_meanStdGroup(meanStdGroup), m_numStdBelow(numStdBelow), m_numStdAbove(numStdAbove) {}

      std::string GetCurrentRange(double& minVal, double& maxVal) override;

    protected:
      std::shared_ptr<imageStatistics::MeanStdGroup> m_meanStdGroup; ///< The mean and standard deviation group.
      double m_numStdBelow; ///< The number of standard deviations below the mean.
      double m_numStdAbove; ///< The number of standard deviations above the mean.
    };

  } // namespace render
} // namespace asdp
