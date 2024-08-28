/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#include "CaptureRealSense.hpp"
#include "calibration/Cameras.hpp"

#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <point_clouds/PointCloudUtilities.hpp>
#include <pcl/filters/voxel_grid.h>

namespace diarc {
  namespace capture {

    CaptureRealSense::CaptureRealSense(const std::string &configFile)
            : Capture(configFile) {

      depthFrame.create(imgHeight, imgWidth, CV_32FC1);

      cfg.enable_stream(RS2_STREAM_COLOR, imgWidth, imgHeight, RS2_FORMAT_RGB8, 30);
      cfg.enable_stream(RS2_STREAM_DEPTH);

      // Create a pipeline to easily configure and start the camera
      //Calling pipeline's start() without any additional parameters will start the first device
      // with its default streams.
      //The start function returns the pipeline profile which the pipeline used to start the device
      rs2::pipeline_profile profile = pipeline.start(cfg);

      // Set up some filters. Found experimentally via playing with values in Realsense Viewer software.
      dec_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, 2);

      temp_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.3f);
      temp_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, 18);

      spat_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, 1);
      spat_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.53f);
      spat_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, 18);

      //Pipeline could choose a device that does not have a color stream
      //If there is no color stream, choose to align depth to another stream
      align_to = find_stream_to_align(profile.get_streams());

      // Create a rs2::align object.
      // rs2::align allows us to perform alignment of depth frames to others frames
      //The "align_to" is the stream type to which we plan to align depth frames.
      align = new rs2::align(align_to);
    };

    CaptureRealSense::~CaptureRealSense() {

    };

    bool CaptureRealSense::captureCurrentCameraFrame() {
      // Using the align object, we block the application until a frameset is available
      rs2::frameset frameset = pipeline.wait_for_frames();
      frameset= dec_filter.process(frameset);
      frameset= spat_filter.process(frameset);
      frameset= temp_filter.process(frameset);
      rs2_extrinsics extrinsics = frameset.get_depth_frame().get_profile().get_extrinsics_to(
              frameset.get_color_frame().get_profile());

      if (logger->isDebugEnabled()) {
        rs2_extrinsics extrinsics = frameset.get_depth_frame().get_profile().get_extrinsics_to(
                frameset.get_color_frame().get_profile());

        LOG4CXX_DEBUG(logger, boost::format("rotation: \n[%f %f %f\n %f %f %f\n %f %f %f]") %
                             extrinsics.rotation[0] % extrinsics.rotation[1] % extrinsics.rotation[2] %
                             extrinsics.rotation[3] % extrinsics.rotation[4] % extrinsics.rotation[5] %
                             extrinsics.rotation[6] % extrinsics.rotation[7] % extrinsics.rotation[8]);

        LOG4CXX_DEBUG(logger, boost::format("translation: [%f %f %f]") %
                             extrinsics.translation[0] % extrinsics.translation[1] % extrinsics.translation[2]);
      }

      // rs2::pipeline::wait_for_frames() can replace the device it uses in case of device error or disconnection.
      // Since rs2::align is aligning depth to some other stream, we need to make sure that the stream was not changed
      //  after the call to wait_for_frames();
      //if (profile_changed(pipeline.get_active_profile().get_streams(), profile.get_streams())) {
      //  //If the profile was changed, update the align object, and also get the new device's depth scale
      //  profile = pipe.get_active_profile();
      //  align_to = find_stream_to_align(profile.get_streams());
      //  align = rs2::align(align_to);
      //  depth_scale = get_depth_scale(profile.get_device());
      //}

      //Get processed aligned frame
      auto
              processed = align->process(frameset);

      // Trying to get both other and aligned depth frames
      rs2::video_frame color_frame = processed.first(align_to);
      rs2::depth_frame aligned_depth_frame = processed.get_depth_frame();

      //If one of them is unavailable, continue iteration
      if (!aligned_depth_frame || !color_frame) {
        return false;
      }

      // Filling frame with RGB info
      cv::Mat color_mat = frame_to_mat(color_frame);
      color_mat.copyTo(frame);

      // Filling depth frame with information from captured and aligned depth
      cv::Mat depth_mat = frame_to_mat(aligned_depth_frame);
      depth_mat.copyTo(depthFrame);

      // create point clouds
      auto points = pc.calculate(aligned_depth_frame);
      pc.map_to(color_frame);
      points_to_pcl(points, color_frame);
      points_to_pcl(points);

      return true;
    }

    float CaptureRealSense::get_depth_scale(rs2::device dev) {
      // Go over the device's sensors
      for (rs2::sensor &sensor : dev.query_sensors()) {
        // Check if the sensor if a depth sensor
        if (rs2::depth_sensor dpt = sensor.as<rs2::depth_sensor>()) {
          return dpt.get_depth_scale();
        }
      }
      throw std::runtime_error("Device does not have a depth sensor");
    }

    rs2_stream CaptureRealSense::find_stream_to_align(const std::vector <rs2::stream_profile> &streams) {
      //Given a vector of streams, we try to find a depth stream and another stream to align depth with.
      //We prioritize color streams to make the view look better.
      //If color is not available, we take another stream that (other than depth)
      rs2_stream align_to = RS2_STREAM_ANY;
      bool depth_stream_found = false;
      bool color_stream_found = false;
      for (rs2::stream_profile sp : streams) {
        rs2_stream profile_stream = sp.stream_type();
        if (profile_stream != RS2_STREAM_DEPTH) {
          if (!color_stream_found)         //Prefer color
            align_to = profile_stream;

          if (profile_stream == RS2_STREAM_COLOR) {
            color_stream_found = true;
          }
        } else {
          depth_stream_found = true;
        }
      }

      if (!depth_stream_found)
        throw std::runtime_error("No Depth stream available");

      if (align_to == RS2_STREAM_ANY)
        throw std::runtime_error("No stream found to align with Depth");

      return align_to;
    }

    // Get RGB values based on normals - texcoords, normals value [u v]
    std::tuple <uint8_t, uint8_t, uint8_t>
    CaptureRealSense::get_texcolor(rs2::video_frame texture, rs2::texture_coordinate texcoords) {
      const int w = texture.get_width(), h = texture.get_height();

      // convert normals [u v] to basic coords [x y]
      int x = std::min(std::max(int(texcoords.u * w + .5f), 0), w - 1);
      int y = std::min(std::max(int(texcoords.v * h + .5f), 0), h - 1);

      int idx = x * texture.get_bytes_per_pixel() + y * texture.get_stride_in_bytes();
      const auto texture_data = reinterpret_cast<const uint8_t *>(texture.get_data());
      return std::tuple<uint8_t, uint8_t, uint8_t>(texture_data[idx], texture_data[idx + 1], texture_data[idx + 2]);
    }

    // Fills in the CLOUDRGB Frame
    void CaptureRealSense::points_to_pcl(const rs2::points &points, const rs2::video_frame &color) {
      auto sp = points.get_profile().as<rs2::video_stream_profile>();

      // Config of PCL Cloud object
      cloudRGB->width = static_cast<uint32_t>(sp.width());
      cloudRGB->height = static_cast<uint32_t>(sp.height());
      cloudRGB->is_dense = false;
      cloudRGB->points.resize(points.size());

      auto tex_coords = points.get_texture_coordinates();
      auto vertices = points.get_vertices();

      // Iterating through all points and setting XYZ coordinates
      // and RGB values
      for (int i = 0; i < points.size(); i++) {
        cloudRGB->points[i].x = vertices[i].x;
        cloudRGB->points[i].y = vertices[i].y;
        cloudRGB->points[i].z = vertices[i].z;

        std::tuple <uint8_t, uint8_t, uint8_t> current_color;
        current_color = get_texcolor(color, tex_coords[i]);

        // Reversed order- 2-1-0 because of BGR model used in camera
        cloudRGB->points[i].r = std::get<2>(current_color);
        cloudRGB->points[i].g = std::get<1>(current_color);
        cloudRGB->points[i].b = std::get<0>(current_color);
      }
    }

    // Fills in Cloud Frame
    void CaptureRealSense::points_to_pcl(const rs2::points &points) {
      auto sp = points.get_profile().as<rs2::video_stream_profile>();

      // Config of PCL Cloud object
      cloud->width = static_cast<uint32_t>(sp.width());
      cloud->height = static_cast<uint32_t>(sp.height());
      cloud->is_dense = false;
      cloud->points.resize(points.size());

      auto vertices = points.get_vertices();
      // Iterating through all points and setting XYZ coordinates
      // and RGB values
      for (int i = 0; i < points.size(); i++) {
        cloud->points[i].x = vertices[i].x;
        cloud->points[i].y = vertices[i].y;
        cloud->points[i].z = vertices[i].z;
      }
    }

    // Convert rs2::frame to cv::Mat
    cv::Mat CaptureRealSense::frame_to_mat(const rs2::frame &f) {
      auto vf = f.as<rs2::video_frame>();
      const int w = vf.get_width();
      const int h = vf.get_height();

      if (f.get_profile().format() == RS2_FORMAT_BGR8) {
        return cv::Mat(cv::Size(w, h), CV_8UC3, (void *) f.get_data(), cv::Mat::AUTO_STEP);
      } else if (f.get_profile().format() == RS2_FORMAT_RGB8) {
        auto r = cv::Mat(cv::Size(w, h), CV_8UC3, (void *) f.get_data(), cv::Mat::AUTO_STEP);
        cv::cvtColor(r, r, cv::COLOR_RGB2BGR);
        return r;
      } else if (f.get_profile().format() == RS2_FORMAT_Z16) {
        return cv::Mat(cv::Size(w, h), CV_16UC1, (void *) f.get_data(), cv::Mat::AUTO_STEP);
      } else if (f.get_profile().format() == RS2_FORMAT_Y8) {
        return cv::Mat(cv::Size(w, h), CV_8UC1, (void *) f.get_data(), cv::Mat::AUTO_STEP);
      }

      throw std::runtime_error("Frame format is not supported yet!");
    }

  }
}

