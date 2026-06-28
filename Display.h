/*
 * Copyright (C) 2024: Arizona Board of Regents on Behalf of the University of Arizona
 */

 /**
  * @file Display.h
  * @brief Apache Strap-Down Pilotage Render/Display submodule header file.
  *
 * @author ReliaSolve.
 * @date May 2, 2024.
 */

#pragma once

#include <thread>
#include <atomic>
#include <cstdint>
#include <chrono>
#include <string>
#include <mutex>
#include <memory>
#include <ASDP_Core_API.h>
#include <Composite.h>
#include <RenderTimingInfo.h>

namespace asdp {
  namespace render {

    /// @brief Event-handling class that passes asynchronous events to the caller.
    struct EventHandlers {
      /// Request that the play/pause state be changed to match nowPlaying.
      /// (See SetNowPlaying() comments for details of pause/replay implementation.)
      /// @param nowPlaying True to play/resume, false to pause.
      void (*ChangePlayPause)(bool nowPlaying, void *userData) = nullptr;

      /// Copy the depth maps for the cameras given the next frame time (center pixel time).
      /// (Only used with CompositeCameras when we have a DepthEstimator.)
      /// @param nextFrameTime The time of the next frame to be rendered (center pixel time).
      void (*CopyDepthInfo)(Time nextFrameTime, void *userData) = nullptr;

      /// Set the depth scale for the cameras to be rendered so that they show depth.
      /// (Only used with CompositeCameras when we have a DepthEstimator.)
      /// @param renderDepth True to render depth, false to render normal imagery.
      void (*SetToRenderDepth)(bool renderDepth, void *userData) = nullptr;

      /// Increment the camera whose values are controlled by offset-adjustment events.
      void (*IncrementActiveCamera)(void* userData) = nullptr;

      /// Decrement the camera whose values are controlled by offset-adjustment events.
      void (*DecrementActiveCamera)(void* userData) = nullptr;

      /// Adjust the color offset calibration values for the active camera.
      /// @param offsetDelta The amount to adjust the offset by, adding it (may be positive or negative).
      void (*AdjustActiveCameraOffset)(int offsetDelta, void* userData) = nullptr;

      /// Adjust the color gain calibration values for the active camera.
      /// @param gainDelta The amount to adjust the gain by, multiplying by the value (may be < or > 1).
      void (*AdjustActiveCameraGain)(float gainDelta, void* userData) = nullptr;

      /// Automatically update the color offsets for all cameras based on point correspondences.
      void (*AutoUpdateColorOffsets)(void* userData) = nullptr;

      /// Automatically update the color offsets and gains for all cameras based on point correspondences.
      void (*AutoUpdateColorOffsetsAndGains)(void* userData) = nullptr;

      /// Save the current camera configuration settings to a file.
      void (*SaveCameraConfig)(const std::string& fileName, void* userData) = nullptr;

      /// Turn on or off annotations showing the camera names.
      void (*ShowCameraNames)(bool showNames, void* userData) = nullptr;

      /// Reset analysis connections, clearing any accumulated annotations.
      void (*ResetAnalysis)(void* userData) = nullptr;
    };

    /// @brief Display base class that defines the interface that all Displays use.
    class Display {
    public:
      /// @brief Constructor
      /// @details Constructs an OpenGL context to be used for rendering, enabling sharing of
      /// textures with other contexts.  Starts a thread to do its rendering, using the
      /// Composite passed in to generate the textured geometry.
      /// @param client The CoreClient used to communicate with the Core to cause software triggers,
      /// a null pointer for none.
      /// @param triggerID The ID of the trigger to use to trigger the cameras, 1 or higher.  If set to 0,
      /// no triggers will be sent to the cameras.
      /// @param triggerAheadMicroseconds The offset in microseconds to subtract from the time of render start.
      /// This is to ensure that the frames make it all the way through the Composite object before being needed.
      /// It is expected to be read from a configuration file and tuned for the specific hardware and software.
      /// @param viewpointOffset The offset from the camera to the pilot head location in the helicopter
      ///        frame of reference, in meters.
      /// @param viewpointRotation The rotation from the camera to the pilot head location in the helicopter
      ///        frame of reference, in degrees.  This rotation is applied around the center of the camera,
      ///        with translation applied after rotation.  The order of rotation is roll, pitch, yaw (x, y, z).
      /// @param depthAheadMicroseconds The offset in microseconds to subtract from the time of render start.
      /// @param composite The Composite used to generate textured geometry.
      Display(std::shared_ptr<Composite> composite,
        std::shared_ptr<CoreClient> client, uint8_t triggerID, uint32_t triggerAheadMicroseconds,
        uint32_t depthAheadMicroseconds, std::array<float, 3> viewpointOffset = { 0, 0, 0 },
        std::array<float, 3> viewpointRotation = { 0, 0, 0 },
        std::shared_ptr<EventHandlers> handlers = nullptr, void* userData = nullptr);

      /// @brief Destructor, virtual so that derived classes can have their destructors called from pointers.
      virtual ~Display();

      /// @brief Cause the object to shut down any threads and release any resources.
      /// @details The base-class function will set m_done and join the display thread and then
      /// release all of its internal resources (including shared pointers).
      /// Derived classes should override this function to release any resources they have allocated.
      /// @return True on success, false on failure (threads may still be running).
      virtual bool Quit();

      /// @brief Pause or resume replay.
      /// @details The play/pause mechanism is fairly involved.  When a Display class wants to pause
      /// or resume replay, it calls teh ChangePlayPause() event handler, if any, to notify the caller.
      /// The caller than tells a connected Storage Server (if any) to pause or resume the replay.
      /// The Storage Server then sends a message to the Core to pause or resume the replay.  The
      /// caller then calls this function, SetNowPlaying(), to let all Displays know their new state.
      /// @param nowPlaying True to play/resume, false to pause.
      virtual void SetNowPlaying(bool nowPlaying);

      /// @brief Get the status of the Display.
      /// @return A string describing the status of the Display.  It is empty when the diplay is
      /// running and has no errors.  It contains "Done" when its window has been closed normally.
      /// It contains an error message when there is an error.
      std::string GetStatus() const;

      /// @brief Make the OpenGL context used by this Display current on the calling thread.
      /// @details This will stop rendering from happening on the Display object until the context
      /// is returned by calling ReturnContext().  It is expected to be called by a thread that wants
      /// to create a context that shares objects with this Display object.
      /// @return True on success, false on failure.
      bool BorrowContext();

      /// @brief Return the OpenGL context used by this Display to its display thread.
      /// @details This will allow rendering to happen on the Display object again.  It detaches the
      /// context from the calling thread and re-attaches it to the display thread.
      /// @return True on success, false on failure.
      bool ReturnContext();

protected:

      /// Viewpoint offset from the camera to the pilot head location in the helicopter frame of reference, in meters.
      /// This is used for rendering the camera views in the correct location relative to the pilot head.
      /// Filled in by the constructor, but can be changed by derived classes if needed.
      std::array<float, 3> m_viewpointOffset;

      /// Viewpoint rotation from the camera to the pilot head location in the helicopter frame of reference,
      /// in degrees. This rotation is applied around the center of the camera, with translation applied
      /// after rotation.
      std::array<float, 3> m_viewpointRotation;

      /// Compositor to use, filled in by the constructor.
      std::shared_ptr<Composite> m_composite;

      /// Event handlers, if any.
      std::shared_ptr<EventHandlers> m_eventHandlers;
      void* m_userData;

      /// @brief Flag to indicate whether the display is currently playing.
      bool m_nowPlaying;

      /// @brief Flag to indicate whether we are rendering camera names.
      bool m_showCameraNames;

      std::shared_ptr<CoreClient> m_client; ///< CoreClient to use, filled in by the constructor.
      uint8_t m_triggerID; ///< Trigger ID to use, filled in by the constructor.
      uint32_t m_offsetMicroseconds; ///< Offset in microseconds to subtract from the time of render start.

      /// Offset in microseconds to subtract from the time of render start.
      uint32_t m_depthAheadMicroseconds;

      /// Timer from the client object, filled in by the constructor
      std::shared_ptr<Timer> m_timer;

      /// Thread to do the rendering.  Started by subclass constructor.  Stopped by Quit() function.
      std::thread m_displayThread;
      /// Flag to signal threads to quit.  Used by Quit().
      std::atomic_bool m_done; 

      std::string m_status = ""; ///< Status of the Display.  Should be set by derived classes.

      /// Mutex to ensure that only one entity is trying to create a window/context at a time
      /// This avoids race conditions due to global state in the window-management library.
      static std::mutex m_windowMutex;

      /// @brief Trigger the cameras to take a picture at the given time.
      /// @details This function is called by the derived class to trigger the cameras to take a picture.
      /// It handles conversion from the time_point to the time format expected by the Core.
      /// The caller sets the time when the textures are needed and the base class handles making adjustments
      /// to ensure that there is enough offset for the frames to make it all the way through to
      /// the Composite object before being needed.  It is not expected to be overriden
      /// by derived classes, but it can be if needed.
      /// @param when The time at which to take the picture.
      /// @return True on success, false on failure.
      virtual bool TriggerCameras(std::chrono::steady_clock::time_point when);

      /// Opaque class used to enable not requiring the application to #include all headers.
      class DisplayImpl;
      /// Instance of the implementation class used to store data.  Filled in by the constructor.
      std::unique_ptr<DisplayImpl> m_impl;

      friend class DisplayWindow;
      friend class DisplayTexture;
      friend class DisplayOpenXR;
      friend class DisplayXSight;
    };

    /// @brief Display class that handles writing to textures, not actually displaying.
    /// @details This class basically produces an OpenGL context that is shared with another
    /// Display object, allowing a thread to render to a texture that can be used by the other
    /// Display object to display the rendered scene.
    class DisplayTexture : public Display {
    public:
      /// @brief Constructor
      /// @param sharedWindow A pointer to a DisplayWindow to share the OpenGL objects with, null if it
      /// is to be the base object to be shared.
      DisplayTexture(Display* sharedWindow = nullptr);

      ~DisplayTexture();

    private:

      /// Opaque class used to enable not requiring the application to #include all headers.
      class DisplayTextureImpl;

      /// Instance of the implementation class used to store data.  Filled in by the constructor.
      std::unique_ptr<DisplayTextureImpl> m_impl;
    };

    /// @brief Display class that displays to a window, perhaps full-screen.
    class DisplayWindow : public Display {
    public:
      /// @brief Constructor
      /// @param windowName The name of the window to create.
      /// @param composite The Composite used to generate textured geometry.  The DisplayWindow object will
      /// reset this pointer just before closing the window, and the caller should reset the pointer passed
      /// in here so that it will be destroyed before the window closes.
      /// @param client The CoreClient used to communicate with the Core to cause software triggers.
      /// @param triggerID The ID of the trigger to use to trigger the cameras.
      /// @param triggerAheadMicroseconds The offset in microseconds to subtract from the time of render start.
      /// This is to ensure that the frames make it all the way through the Composite object before being needed.
      /// It is expected to be read from a configuration file and tuned for the specific hardware and software.
      /// @param depthAheadMicroseconds The offset in microseconds to subtract from the time of render start.
      /// @param viewpointOffset The offset from the camera to the pilot head location in the helicopter
      ///        frame of reference, in meters.
      /// @param viewpointRotation The rotation from the camera to the pilot head location in the helicopter
      ///        space, in degrees.  This rotation is applied around the center of the camera, with translation
      ///        applied after rotation.
      /// @param fps The number of frames per second requested for a full-screen window.  The system will busy-wait
      /// to achieve at most this frame rate.  For full-screen windows, this is the frame rate we ask for
      /// on the monitor.  For windows that are not full screen, this should be set to the actual monitor
      /// refresh rate.
      /// @param renderAheadMicroseconds The number of microseconds ahead of the next swap time to begin
      /// rendering.  This is to ensure that the rendering is done in time for the swap to happen while
      /// providing the minimum prediction interval and delaying as long as possible to enable new frames
      /// to arrive before rendering.  If this value is larger than 1 million / fps, the frames will not
      /// wait before rendering.
      /// @param desiredWidth The width of the display in pixels.
      /// @param desiredHeight The height of the display in pixels.
      /// @param horizontalFOVDegrees The horizontal field of view of the display in degrees.
      /// @param joystick The name of the joystick to use for input, if any.
      /// @param sharedWindow A pointer to a DisplayWindow to share the OpenGL objects with, or nullptr if none.
      /// @param fullScreen True if the display should be full-screen.
      /// @param desiredDisplay The index of the desired display to use if full-screen.
      /// @param hidden True if the window should be hidden.
      /// @param handlers Event handlers to use, if any.
      /// @param userData User data to pass to the event handlers.
      /// @param timingInfo Timing information on display operations should be stored here, if it is not null.
      /// @param replaying True if the display is replaying a recording, false if not.
      DisplayWindow(std::string windowName, std::shared_ptr<Composite> composite,
        std::shared_ptr<CoreClient> client, uint8_t triggerID, uint32_t triggerAheadMicroseconds,
        uint32_t depthAheadMicroseconds, std::array<float, 3> viewpointOffset = { 0, 0, 0 },
        std::array<float, 3> viewpointRotation = { 0, 0, 0 },
        float fps = 60, uint32_t renderAheadMicroseconds = 2500,
        int desiredWidth = 1280, int desiredHeight = 1024, float horizontalFOVDegrees = 90.0,
        std::string joystick = "", Display *sharedWindow = nullptr,
        bool fullScreen = false, int desiredDisplay = 0, bool hidden = false,
        std::shared_ptr<EventHandlers> handlers = nullptr, void* userData = nullptr,
        RenderTimingInfo* timingInfo = nullptr, bool replaying = false);

      void SetNowPlaying(bool nowPlaying) override;

      ~DisplayWindow();

    private:
      /// Where to store render timing info, if not nullptr.
      RenderTimingInfo* m_timingInfo;

      /// Are we replaying?
      bool m_replaying;

      /// Pointer to time when we started pausing, nullptr if not pausing.
      std::unique_ptr<Time> m_pauseTime;

      /// @brief Method to implement the display thread.
      void DisplayThread(std::string windowName,
        float fps, uint32_t renderAheadMicroseconds,
        int desiredWidth, int desiredHeight, float horizontalFOVDegrees,
        std::string joystick, Display* sharedWindow,
        bool fullScreen, int desiredDisplay, bool hidden);

      /// @brief Helper function to set the viewport dimensions based on the window size.
      /// @param viewInfo The ViewRenderInfo object to set the viewport size and field of view in.
      /// @param width The width of the window in pixels, 0 to use the current width.
      /// @param height The height of the window in pixels, 0 to use the current height.
      void SetViewportSizeAndFOVs(ViewRenderInfo &viewInfo, int width = 0, int height = 0);

      /// @brief Helper function to handle keyboard input.
      void HandleKeyboard();

      /// @brief Helpfer function to handle mouse input.
      void HandleMouse();

      /// @brief Helper function to clamp the viewing orientation to be within the expected visible range.
      /// @details This function is called by the display thread to ensure that the view orientation is
      /// within the expected range.  It is expected to be called after the view orientation is updated.
      /// It also converts from the specified amount of rotation about Z and X into an Euler angle that
      /// describes the entire transformation.
      void ComputeAndClampViewOrientation();

      /// Opaque class used to enable not requiring the application to #include all headers.
      class DisplayWindowImpl;
      /// Instance of the implementation class used to store data.  Filled in by the constructor.
      std::unique_ptr<DisplayWindowImpl> m_impl;
    };

    /// @brief Display class that displays using OpenXR.
    class DisplayOpenXR : public Display {
    public:
      /// @brief Constructor
      /// @param composite The Composite used to generate textured geometry.
      /// @param sharedWindow A pointer to a DisplayWindow to share the OpenGL objects with, or nullptr if none.
      /// @param client The CoreClient used to communicate with the Core to cause software triggers,
      /// a null pointer for none.
      /// @param triggerID The ID of the trigger to use to trigger the cameras.
      /// @param triggerAheadMicroseconds The offset in microseconds to subtract from the time of render start.
      /// This is to ensure that the frames make it all the way through the Composite object before being needed.
      /// It is expected to be read from a configuration file and tuned for the specific hardware and software.
      /// @param depthAheadMicroseconds The offset in microseconds to subtract from the time of render start.
      /// @param viewpointOffset The offset from the camera to the pilot head location in the helicopter
      ///        frame of reference, in meters.
      /// @param viewpointRotation The rotation from the camera to the pilot head location in the helicopter
      ///        frame of reference, in degrees.  This rotation is applied around the center of
      ///        the camera, with translation applied after rotation.  The order of rotation is roll,
      ///        pitch, yaw (x, y, z).
      /// @param renderAheadMicroseconds The number of microseconds ahead of the next swap time to begin
      /// rendering.  This is to ensure that the rendering is done in time for the swap to happen while
      /// providing the minimum prediction interval and delaying as long as possible to enable new frames
      /// to arrive before rendering.  If this value is larger than 1 million / fps, the frames will not
      /// wait before rendering.
      /// @param verbosity The level of verbosity to use in the OpenXR API, 0 for none.
      /// @param handlers Event handlers to use, if any.
      /// @param userData User data to pass to the event handlers.
      /// @param timingInfo Timing information on display operations should be stored here, if it is not null.
      /// @param replaying True if the display is replaying a recording, false if not.
      DisplayOpenXR(std::shared_ptr<Composite> composite, Display* sharedWindow,
        std::shared_ptr<CoreClient> client, uint8_t triggerID, uint32_t triggerAheadMicroseconds,
        uint32_t depthAheadMicroseconds, std::array<float, 3> viewpointOffset = { 0, 0, 0 },
        std::array<float, 3> viewpointRotation = { 0, 0, 0 },
        uint32_t renderAheadMicroseconds = 2500, int verbosity = 0,
        std::shared_ptr<EventHandlers> handlers = nullptr, void* userData = nullptr,
        RenderTimingInfo* timingInfo = nullptr, bool replaying = false);

      void SetNowPlaying(bool nowPlaying) override;

      ~DisplayOpenXR();

    private:
      /// Where to store render timing info, if not nullptr.
      RenderTimingInfo* m_timingInfo;

      /// Are we replaying?
      bool m_replaying;

      /// @brief Method to implement the display thread.
      void DisplayThread(Display* sharedWindow, uint32_t renderAheadMicroseconds);

      /// Opaque class used to enable not requiring the application to #include all headers.
      class DisplayOpenXRImpl;

      /// Instance of the implementation class used to store data.  Filled in by the constructor.
      std::unique_ptr<DisplayOpenXRImpl> m_impl;
    };

    /// @brief Display class that displays to an Elbit XSight HMD using a full-screen window.
    class DisplayXSight : public Display {
    public:
      /// @brief Constructor
      /// @param NICName The name of the NIC to use listen for UDP packets from the XSight HMD.
      /// @param composite The Composite used to generate textured geometry.  The DisplayXSight object will
      /// reset this pointer just before closing the window, and the caller should reset the pointer passed
      /// in here so that it will be destroyed before the window closes.
      /// @param client The CoreClient used to communicate with the Core to cause software triggers.
      /// @param triggerID The ID of the trigger to use to trigger the cameras.
      /// @param triggerAheadMicroseconds The offset in microseconds to subtract from the time of render start.
      /// This is to ensure that the frames make it all the way through the Composite object before being needed.
      /// It is expected to be read from a configuration file and tuned for the specific hardware and software.
      /// @param depthAheadMicroseconds The offset in microseconds to subtract from the time of render start.
      /// @param viewpointOffset The offset from the camera to the pilot head location in the helicopter
      ///        frame of reference, in meters.
      /// @param viewpointRotation The rotation from the camera to the pilot head location in the helicopter
      ///        frame of reference, in degrees.  This rotation is applied around the center of
      ///        the camera, with translation applied after rotation.  The order of rotation is roll,
      ///        pitch, yaw (x, y, z).
      /// @param desiredDisplay The index of the desired display to use (0 = first, default 1).
      /// @param desiredWidth The width of the display in pixels.  This must be a multiple of two because the
      /// data is encoded as two monochrome values per color pixel before being sent to the device.
      /// @param desiredHeight The height of the display in pixels.
      /// @param fps The number of frames per second requested.  The system will busy-wait
      /// to achieve at most this frame rate.  This is the frame rate we ask for on the monitor.
      /// @param horizontalFOVDegrees The horizontal field of view of the display in degrees.  This is the
      /// size of oversized view that will be resampled into the smaller actual field of view to support
      /// latency compensation, so its value must match the one used by the XSight unit expects
      /// but does not need to be a particular value tied to the HMD). We set it here to an even number.
      /// @param renderAheadMicroseconds The number of microseconds ahead of the next swap time to begin
      /// rendering.  This is to ensure that the rendering is done in time for the swap to happen while
      /// providing the minimum prediction interval and delaying as long as possible to enable new frames
      /// to arrive before rendering.  If this value is larger than 1 million / fps, the frames will not
      /// wait before rendering.
      /// @param sharedWindow A pointer to a DisplayWindow to share the OpenGL objects with, or nullptr if none.
      /// @param handlers Event handlers to use, if any.
      /// @param userData User data to pass to the event handlers.
      /// @param timingInfo Timing information on display operations should be stored here, if it is not null.
      /// @param replaying True if the display is replaying a recording, false if not.
      /// @param encodeMonochrome True to encode the data as two monochrome values per color pixel,
      /// false to leave as RGB. Some XSight displays require this and some do not.
      DisplayXSight(std::string NICName, std::shared_ptr<Composite> composite, Display* sharedWindow,
        std::shared_ptr<CoreClient> client, uint8_t triggerID, uint32_t triggerAheadMicroseconds,
        uint32_t depthAheadMicroseconds, std::array<float, 3> viewpointOffset,
        std::array<float, 3> viewpointRotation,
        uint32_t renderAheadMicroseconds = 2500,  ///< @todo Match XSight specs
        std::shared_ptr<EventHandlers> handlers = nullptr, void* userData = nullptr,
        RenderTimingInfo* timingInfo = nullptr, bool replaying = false,
        int desiredDisplay = 1,
        int desiredWidth = 2560, int desiredHeight = 2048, float fps = 50,
        float horizontalFOVDegrees = 70.0f,
        bool encodeMonochrome = true
      );

      void SetNowPlaying(bool nowPlaying) override;

      ~DisplayXSight();

    private:
      /// Name of the NIC to use for UDP packets from the XSight HMD.
      std::string m_NICName;

      /// Where to store render timing info, if not nullptr.
      RenderTimingInfo* m_timingInfo;

      /// Are we replaying?
      bool m_replaying;

      /// Pointer to time when we started pausing, nullptr if not pausing.
      std::unique_ptr<Time> m_pauseTime;

      /// @brief Method to implement the display thread.
      void DisplayThread(
        float fps, uint32_t renderAheadMicroseconds,
        int desiredWidth, int desiredHeight,
        Display* sharedWindow,
        int desiredDisplay);

      /// @brief Helper function to set the viewport dimensions based on the window size.
      /// @param viewInfo The ViewRenderInfo object to set the viewport size and field of view in.
      /// @param width The width of the window in pixels, 0 to use the current width.
      /// @param height The height of the window in pixels, 0 to use the current height.
      void SetViewportSizeAndFOVs(ViewRenderInfo& viewInfo, int width = 0, int height = 0);

      /// Opaque class used to enable not requiring the application to #include all headers.
      class DisplayXSightImpl;
      /// Instance of the implementation class used to store data.  Filled in by the constructor.
      std::unique_ptr<DisplayXSightImpl> m_impl;
    };

  } // namespace render
} // namespace asdp
