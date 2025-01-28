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

#include <vector>
#include <string>
#include <memory>
#include <mutex>
#include <atomic>
#include <Composite.h>
#include <Display.h>

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

      /// @brief Class that computes the mean and standard deviation of pixel values for an set of images.
      class MeanStdGroup {
      public:

        /// @brief Construct the mean and standard deviation calculator for a set of images.
        /// @details The caller must have a valid OpenGL context on the calling thread when calling any of the
        /// functions in this class, including the constructor. This context must have had glewInit() called on it.
        /// This class will start a thread that updates the estimated values on one camera at a time at the
        /// specified interval.  The thread will run until the destructor is called.
        /// It updates its internal mean and standard deviation values at the specified interval in a thread-safe
        /// manner.
        /// @param cameras Cameras to use for the images.
        /// @param display Display to borrow the OpenGL context from.
        /// @param updateInterval Interval in seconds to update the mean and standard deviation values.
        MeanStdGroup(std::vector< std::shared_ptr<asdp::render::CameraRenderInfo> > cameras,
          std::shared_ptr<asdp::render::Display> display,
          double updateInterval = 1.0/60);

        /// @brief Virtual destructor so that proper deconstruction happens on pointers.
        /// @details This will stop the thread that updates the mean and standard deviation values.
        virtual ~MeanStdGroup();

        /// @brief Get the mean and standard deviation of the pixel values in the images.
        /// @param [out] mean The mean of the pixel values in the images.
        /// @param [out] stddev The standard deviation of the pixel values in the images.
        /// @return Empty string on success, error description on failure.
        std::string GetMeanStd(double &mean, double &stddev) const;

        /// @brief Test function that returns an empty string on success or an error message on failure.
        static std::string Test();

      protected:
        std::vector< std::shared_ptr<asdp::render::CameraRenderInfo> > m_cameras; ///< Cameras to use for the images.
        std::shared_ptr<asdp::render::Display> m_display; ///< Display to borrow the OpenGL context from.
        double m_updateInterval;          ///< Interval in seconds to update the mean and standard deviation values.
        std::string m_status;             ///< Status filled in by the constructor and other methods.

        // Maintain vectors of values, one per camera.  These are initialized as they are read from the cameras
        // and then overwritten when all cameras have been read.
        mutable std::mutex m_mutex;       ///< Mutex to protect the mean and standard deviation vectors.
        std::vector< std::shared_ptr<MeanStd> > m_meanStds; ///< Mean and standard deviation calculators for each camera.
        std::vector<double> m_means;      ///< Current estimated mean of the pixel values in each image.
        std::vector<double> m_stds;       ///< Current estimated standard deviation of the pixel values in each image.

        std::thread m_updateThread;       ///< Thread that updates the mean and standard deviation values.
        void UpdateThread();              ///< Thread function that updates the mean and standard deviation values.
        std::atomic_bool m_stopThread;    ///< Flag to stop the thread that updates the mean and standard deviation values.
      };

    } // namespace imageStatistics
  } // namespace render
} // namespace asdp