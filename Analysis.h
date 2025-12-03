/*
 * Copyright (C) 2025: Arizona Board of Regents on Behalf of the University of Arizona
 */

 /**
  * @file Analysis.h
  * @brief Apache Strap-Down Pilotage Render header file for analysis objects.
  *
 * @author ReliaSolve.
 * @date December 2, 2025.
 */

#pragma once

#include <memory>
#include <vector>
#include <array>
#include <string>
#include <cstdint>
#include <ASDP_Core_API.h>
#include <Composite.h>

namespace asdp {
  namespace analysis {

    /// @brief Holds the required and optional entries described in Appendix C of TR-006v29_Software_Architecture.docx
    class AnalysisReport {
    public:
      /// @brief Constructor
      /// @param jsonString JSON string to be parsed to initialize member variables.
      /// @throws std::invalid_argument if the JSON string is malformed or required entries are missing.
      AnalysisReport(std::string jsonString);

      /// @brief Converts the AnalysisReport into an annotation for rendering.
      /// @param chanceThreshold Minimum chance value for including a classification in the annotation (range 0-1).
      /// @param alpha Transparency level for the annotation (range 0-1).
      /// @return An annotation. Its label field will be empty if no classifications meet the chance threshold.
      asdp::render::CompositeCameras::Annotation ConvertToAnnotation(float chanceThreshold = 0.0f, float alpha = 1.0f);

      /// @brief Test function for the analysis module.
      /// @return An empty string on success, or an error message on failure.
      static std::string Test();

      /// Required entries
      uint32_t CamID = 0;
      asdp::Time Timestamp = {};
      std::string Name;

      /// Optional entries. Shared pointer value of nullptr indicates absence of the entry.
      std::shared_ptr< std::array<float, 2> > Loc;
      std::shared_ptr< std::array<float, 2> > Rect;
      std::shared_ptr< std::vector< std::array<float, 3> > > Vel;

      /// Possibly empty vector of detected classes
      class Classification {
      public:
        std::shared_ptr<std::string> Type;
        std::shared_ptr<float> Chance;
        std::shared_ptr<std::string> IFF;
      };
      std::vector<Classification> Class;
    };

  } // namespace analysis
} // namespace asdp
