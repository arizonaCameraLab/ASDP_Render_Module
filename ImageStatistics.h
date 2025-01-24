/*
 * Copyright (C) 2024: Arizona Board of Regents on Behalf of the University of Arizona
 */

 /**
  * @file Distortion.h
  * @brief Apache Strap-Down Pilotage Render/Image Statistics classes header file.
  *
  * @author ReliaSolve.
  * @date January 23, 2025.
  */

#pragma once

#include <array>
#include <vector>
#include <string>
#include <memory>
#include <Composite.h>

namespace asdp {
  namespace render {
    namespace imageStatistics {

      /// @brief Class that computes the mean and standard deviation of pixel values for an image.
      class MeanStd {
      public:

        /// @brief Construct the mean and standard deviation calculator.
        /// @details The caller must have a valid OpenGL context on the calling thread when calling any of the
        /// functions in this class, including the constructor. This context must have had glewInit() called on it.
        /// @param camera Camera to use for the image.
        MeanStd(std::shared_ptr<asdp::render::CameraRenderInfo> camera);

        /// @brief Virtual destructor so that proper deconstruction happens on pointers.
        virtual ~MeanStd() = default;

        /// @brief Compute the mean and standard deviation of the pixel values in the image.
        /// @details This function will lock the most-recent image from the camera and compute the mean and
        /// standard deviation of the pixel values in the image.
        /// @param [out] mean The mean of the pixel values in the image.
        /// @param [out] stddev The standard deviation of the pixel values in the image.
        /// @return Empty string on success, error description on failure.
        std::string Compute(double &mean, double &stddev) const;

        /// @brief Get the status of the constructor.
        /// @return An empty string on success or an error message on failure.
        std::string GetConstructorStatus() const { return m_constructorStatus; }

        /// @brief Test function that returns an empty string on success or an error message on failure.
        static std::string Test();

        /// @brief Test function that returns the time to compute the mean and standard deviation of a single image.
        /// @details It computes the mean and standard deviation over many images and returns the average time to compute
        /// the mean and standard deviation of a single image.
        /// @param width Width of the image to test.
        /// @param height Height of the image to test.
        /// @return Time in seconds to compute the mean and standard deviation of a single image.
        static float SpeedTestSingleCalculation(uint16_t width, uint16_t height);

      protected:
        std::string m_constructorStatus; ///< Status of the constructor.
        class MeanStdImpl; ///< Forward declaration of the implementation class.
        std::unique_ptr<MeanStdImpl> m_impl; ///< Pointer to the implementation class.
      };

    } // namespace imageStatistics
  } // namespace render
} // namespace asdp