/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#ifndef V4L2HELPER_HPP
#define V4L2HELPER_HPP

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include <fcntl.h>              /* low-level i/o */
#include <unistd.h>
#include <errno.h>
#include <malloc.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>

#include <asm/types.h>          /* for videodev2.h */

#include <linux/videodev2.h>

#include <opencv2/opencv.hpp>

typedef enum {
	IO_METHOD_READ,
	IO_METHOD_MMAP,
	IO_METHOD_USERPTR,
} io_method;

struct buffer {
        void *                  start;
        size_t                  length;
};

class V4L2Helper {
public:

  V4L2Helper (const int deviceNum, const int imgWidth, const int imgHeight);
  ~V4L2Helper ();

  int xioctl (int fd, int request, void *arg);  
  void errno_msg (const char *s);
  bool open_device ();
  bool close_device ();
  bool init_read (unsigned int buffer_size);
  bool init_mmap (void);
  bool init_userp (unsigned int buffer_size);
  bool init_device ();
  bool uninit_device ();
  bool init_V4L2Device ();
  void cleanup_V4L2Device ();
  
  void get_frame (cv::Mat frameToFill);

private:
    bool start_capture();
    bool stop_capture();
    bool read_frame(cv::Mat v4l2frame);
    void process_image(cv::Mat v4l2frame, const void *p);

    char * dev_name;// = "/dev/video0";
    io_method io;// = IO_METHOD_MMAP;
    int fd;// = -1;
    struct buffer * buffers;// = NULL;
    unsigned int n_buffers;// = 0;
    int width;
    int height;

#define FMT_TYPE V4L2_BUF_TYPE_VIDEO_CAPTURE
//#define FMT_PIX_WIDTH 320
//#define FMT_PIX_HEIGHT 240
#define FMT_PIX_PIXELFORMAT V4L2_PIX_FMT_YUYV
#define FMT_PIX_FIELD V4L2_FIELD_INTERLACED
#define CLEAR(x) memset (&(x), 0, sizeof (x))

};

#endif
