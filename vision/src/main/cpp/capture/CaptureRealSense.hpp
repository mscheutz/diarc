/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#include "Capture.hpp"
#include <pcl/point_types.h>
#include <pcl/point_types.h>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>

namespace ade {
  namespace capture {

    class CaptureRealSense : public Capture {
    public:

      CaptureRealSense(const std::string &configFile);

      ~CaptureRealSense();

      bool captureCurrentCameraFrame();

    private:

      //! Get RGB values based on normals - texcoords, normals value [u v]
      std::tuple<uint8_t, uint8_t, uint8_t> get_texcolor(rs2::video_frame texture, rs2::texture_coordinate texcoords);

      //! Fills in the CLOUDRGB Frame
      void points_to_pcl(const rs2::points &points, const rs2::video_frame &color);

      //! Fills in Cloud Frame
      void points_to_pcl(const rs2::points &points);

      //! Convert rs2::frame to cv::Mat
      cv::Mat frame_to_mat(const rs2::frame &f);

      //! get the depth scale -- currently not needed for the D435 sensor
      float get_depth_scale(rs2::device dev);

      //! find a suitable non-depth stream to align to -- should be color stream
      rs2_stream find_stream_to_align(const std::vector<rs2::stream_profile>& streams);

      rs2_stream align_to;
      rs2::align* align;
      rs2::pipeline pipeline;
      rs2::pipeline_profile profile;
      rs2::pointcloud pc;
      rs2::config cfg;
      rs2::decimation_filter dec_filter;
      rs2::spatial_filter spat_filter;
      rs2::temporal_filter temp_filter;
    };
  }
}

