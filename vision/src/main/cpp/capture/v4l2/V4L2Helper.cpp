/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#include <iostream>
//#include <cstdlib>
#include <sstream>

#include "V4L2Helper.hpp"

//typedef enum {
//	IO_METHOD_READ,
//	IO_METHOD_MMAP,
//	IO_METHOD_USERPTR,
//} io_method;
//
//struct buffer {
//        void *                  start;
//        size_t                  length;
//};
//
//static char *           dev_name        = "/dev/video0";
//static io_method	io		= IO_METHOD_MMAP;
//static int              fd              = -1;
//struct buffer *         buffers         = NULL;
//static unsigned int     n_buffers       = 0;

V4L2Helper::V4L2Helper(const int deviceNum, const int imgWidth, const int imgHeight)
        : io(IO_METHOD_MMAP),
          fd(-1),
          buffers(NULL),
          n_buffers(0),
          width(imgWidth),
          height(imgHeight) {
  std::stringstream ss; //create a stringstream
  ss << deviceNum; //add number to the stream
  std::string str("/dev/video" + ss.str());

  dev_name = new char[str.size() + 1];
  strcpy(dev_name, str.c_str());
}

V4L2Helper::~V4L2Helper() {
  cleanup_V4L2Device();
  delete[] dev_name;
}

int V4L2Helper::xioctl(int fd, int request, void *arg) {
  int r;

  do r = ioctl(fd, request, arg);
  while (-1 == r && EINTR == errno);

  return r;
}

void V4L2Helper::errno_msg(const char *s) {
  fprintf(stderr, "%s error %d, %s.\n", s, errno, strerror(errno));
}

bool V4L2Helper::open_device() {
  struct stat st;

  if (-1 == stat(dev_name, &st)) {
    fprintf(stderr, "Cannot identify '%s': %d, %s.\n",
            dev_name, errno, strerror(errno));
    return false;
  }

  if (!S_ISCHR (st.st_mode)) {
    fprintf(stderr, "%s is no device.\n", dev_name);
    return false;
  }

  fd = open(dev_name, O_RDWR /* required */ | O_NONBLOCK, 0);

  if (-1 == fd) {
    fprintf(stderr, "Cannot open '%s': %d, %s.\n",
            dev_name, errno, strerror(errno));
    return false;
  }

  return true;
}

bool V4L2Helper::close_device() {
  if (-1 == close(fd)) {
    errno_msg("close");
    return false;
  }

  fd = -1;
  return true;
}


bool V4L2Helper::init_read(unsigned int buffer_size) {
  buffers = (buffer *) calloc(1, sizeof(*buffers));

  if (!buffers) {
    fprintf(stderr, "Out of memory.\n");
    return false;
  }

  buffers[0].length = buffer_size;
  buffers[0].start = malloc(buffer_size);

  if (!buffers[0].start) {
    fprintf(stderr, "Out of memory.\n");
    return false;
  }

  return true;
}

bool V4L2Helper::init_mmap(void) {
  struct v4l2_requestbuffers req;

  CLEAR (req);

  req.count = 4;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_MMAP;

  if (-1 == xioctl(fd, VIDIOC_REQBUFS, &req)) {
    if (EINVAL == errno) {
      fprintf(stderr, "%s does not support "
                      "memory mapping\n", dev_name);
      return false;
    } else {
      errno_msg("VIDIOC_REQBUFS");
      return false;
    }
  }

  if (req.count < 2) {
    fprintf(stderr, "Insufficient buffer memory on %s\n",
            dev_name);
    return false;
  }

  buffers = (buffer *) calloc(req.count, sizeof(*buffers));

  if (!buffers) {
    fprintf(stderr, "Out of memory\n");
    return false;
  }

  for (n_buffers = 0; n_buffers < req.count; ++n_buffers) {
    struct v4l2_buffer buf;

    CLEAR (buf);

    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = n_buffers;

    if (-1 == xioctl(fd, VIDIOC_QUERYBUF, &buf)) {
      errno_msg("VIDIOC_QUERYBUF");
      return false;
    }


    buffers[n_buffers].length = buf.length;
    buffers[n_buffers].start =
            mmap(NULL /* start anywhere */,
                 buf.length,
                 PROT_READ | PROT_WRITE /* required */,
                 MAP_SHARED /* recommended */,
                 fd, buf.m.offset);

    if (MAP_FAILED == buffers[n_buffers].start) {
      errno_msg("mmap");
      return false;
    }
  }

  return true;
}

bool V4L2Helper::init_userp(unsigned int buffer_size) {
  struct v4l2_requestbuffers req;
  unsigned int page_size;

  page_size = getpagesize();
  buffer_size = (buffer_size + page_size - 1) & ~(page_size - 1);

  CLEAR (req);

  req.count = 4;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_USERPTR;

  if (-1 == xioctl(fd, VIDIOC_REQBUFS, &req)) {
    if (EINVAL == errno) {
      fprintf(stderr, "%s does not support "
                      "user pointer i/o\n", dev_name);
      return false;
    } else {
      errno_msg("VIDIOC_REQBUFS");
      return false;
    }
  }

  buffers = (buffer *) calloc(4, sizeof(*buffers));

  if (!buffers) {
    fprintf(stderr, "Out of memory.\n");
    return false;
  }

  for (n_buffers = 0; n_buffers < 4; ++n_buffers) {
    buffers[n_buffers].length = buffer_size;
    buffers[n_buffers].start = memalign(/* boundary */ page_size,
                                                       buffer_size);

    if (!buffers[n_buffers].start) {
      fprintf(stderr, "Out of memory.\n");
      return false;
    }
  }
  return true;
}

bool V4L2Helper::init_device() {

  struct v4l2_capability cap;
  struct v4l2_cropcap cropcap;
  struct v4l2_crop crop;
  struct v4l2_format fmt;
  unsigned int min;

  if (-1 == xioctl(fd, VIDIOC_QUERYCAP, &cap)) {
    if (EINVAL == errno) {
      fprintf(stderr, "%s is no V4L2 device.\n",
              dev_name);
      return false;
    } else {
      errno_msg("VIDIOC_QUERYCAP");
      return false;
    }
  }

  if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
    fprintf(stderr, "%s is no video capture device.\n",
            dev_name);
    return false;
  }

  switch (io) {
    case IO_METHOD_READ:
      if (!(cap.capabilities & V4L2_CAP_READWRITE)) {
        fprintf(stderr, "%s does not support read i/o.\n",
                dev_name);
        return false;
      }

      break;

    case IO_METHOD_MMAP:
    case IO_METHOD_USERPTR:
      if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
        fprintf(stderr, "%s does not support streaming i/o.\n",
                dev_name);
        return false;
      }

      break;
  }


  /* Select video input, video standard and tune here. */


  CLEAR (cropcap);

  cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

  if (0 == xioctl(fd, VIDIOC_CROPCAP, &cropcap)) {
    crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    crop.c = cropcap.defrect; /* reset to default */

    if (-1 == xioctl(fd, VIDIOC_S_CROP, &crop)) {
      switch (errno) {
        case EINVAL:
          /* Cropping not supported. */
          break;
        default:
          /* Errors ignored. */
          break;
      }
    }
  } else {
    /* Errors ignored. */
  }


  CLEAR (fmt);

  fmt.type = FMT_TYPE;
  fmt.fmt.pix.width = width;
  fmt.fmt.pix.height = height;
  fmt.fmt.pix.pixelformat = FMT_PIX_PIXELFORMAT;
  fmt.fmt.pix.field = FMT_PIX_FIELD;

  if (-1 == xioctl(fd, VIDIOC_S_FMT, &fmt)) {
    errno_msg("VIDIOC_S_FMT");
    return false;
  }

  /* Note VIDIOC_S_FMT may change width and height. */

  /* Buggy driver paranoia. */
  min = fmt.fmt.pix.width * 2;
  if (fmt.fmt.pix.bytesperline < min)
    fmt.fmt.pix.bytesperline = min;
  min = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;
  if (fmt.fmt.pix.sizeimage < min)
    fmt.fmt.pix.sizeimage = min;

  switch (io) {
    case IO_METHOD_READ:
      if (!init_read(fmt.fmt.pix.sizeimage))
        return false;
      break;

    case IO_METHOD_MMAP:
      if (!init_mmap())
        return false;
      break;

    case IO_METHOD_USERPTR:
      if (!init_userp(fmt.fmt.pix.sizeimage))
        return false;
      break;
  }

  return true;
}

bool V4L2Helper::uninit_device() {
  unsigned int i;

  switch (io) {
    case IO_METHOD_READ:
      free(buffers[0].start);
      break;

    case IO_METHOD_MMAP:
      for (i = 0; i < n_buffers; ++i)
        if (-1 == munmap(buffers[i].start, buffers[i].length)) {
          errno_msg("munmap");
          return false;
        }

      break;

    case IO_METHOD_USERPTR:
      for (i = 0; i < n_buffers; ++i)
        free(buffers[i].start);
      break;
  }

  free(buffers);

  return true;
}


bool V4L2Helper::start_capture() {
  unsigned int i;
  enum v4l2_buf_type type;

  switch (io) {
    case IO_METHOD_READ:
      /* Nothing to do. */
      break;

    case IO_METHOD_MMAP:
      for (i = 0; i < n_buffers; ++i) {
        struct v4l2_buffer buf;

        CLEAR (buf);

        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        if (-1 == xioctl(fd, VIDIOC_QBUF, &buf)) {
          errno_msg("VIDIOC_QBUF");
          return false;
        }
      }

      type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

      if (-1 == xioctl(fd, VIDIOC_STREAMON, &type)) {
        errno_msg("VIDIOC_STREAMON");
        return false;
      }

      break;

    case IO_METHOD_USERPTR:
      for (i = 0; i < n_buffers; ++i) {
        struct v4l2_buffer buf;

        CLEAR (buf);

        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_USERPTR;
        buf.index = i;
        buf.m.userptr = (unsigned long) buffers[i].start;
        buf.length = buffers[i].length;

        if (-1 == xioctl(fd, VIDIOC_QBUF, &buf)) {
          errno_msg("VIDIOC_QBUF");
          return false;
        }
      }

      type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

      if (-1 == xioctl(fd, VIDIOC_STREAMON, &type)) {
        errno_msg("VIDIOC_STREAMON");
        return false;
      }

      break;
  }

  return true;
}

bool V4L2Helper::stop_capture() {
  enum v4l2_buf_type type;

  switch (io) {
    case IO_METHOD_READ:
      /* Nothing to do. */
      break;

    case IO_METHOD_MMAP:
    case IO_METHOD_USERPTR:
      type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

      if (-1 == xioctl(fd, VIDIOC_STREAMOFF, &type)) {
        errno_msg("VIDIOC_STREAMON");
        return false;
      }

      break;
  }

  return true;
}

bool V4L2Helper::init_V4L2Device() {
  if (!open_device()) {
    fprintf(stderr, "Opening %s device failed.\n",
            dev_name);
    return false;
  }

  if (!init_device()) {
    fprintf(stderr, "Initializing %s device failed.\n",
            dev_name);
    return false;
  }

  if (!start_capture()) {
    fprintf(stderr, "Failed to start capturing from %s.\n",
            dev_name);
    return false;
  } else {
    printf("Capture started from %s\n", dev_name);
  }

  return true;
}

void V4L2Helper::cleanup_V4L2Device() {
  if (!stop_capture()) {
    fprintf(stderr, "Failed to stop capturing from %s.\n",
            dev_name);
  } else {
    printf("%s device has now stopped capturing\n", dev_name);
  }

  if (!uninit_device()) {
    fprintf(stderr, "Uninitializing %s device failed.\n",
            dev_name);
  } else {
    printf("%s device uninitialized.\n", dev_name);
  }

  if (!close_device()) {
    fprintf(stderr, "Closing %s device failed.\n",
            dev_name);
  } else {
    printf("Closed %s\n", dev_name);
  }
}

bool V4L2Helper::read_frame(cv::Mat v4l2frame) {

  struct v4l2_buffer buf;
  unsigned int i;

  switch (io) {
    case IO_METHOD_READ:
      if (-1 == read(fd, buffers[0].start, buffers[0].length)) {
        switch (errno) {
          case EAGAIN:
            return false;

          case EIO:
            /* Could ignore EIO, see spec. */

            /* fall through */

          default:
            errno_msg("read");
            exit(EXIT_FAILURE);
        }
      }

      process_image(v4l2frame, buffers[0].start);

      break;

    case IO_METHOD_MMAP:
      CLEAR (buf);

      buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_MMAP;

      if (-1 == xioctl(fd, VIDIOC_DQBUF, &buf)) {
        switch (errno) {
          case EAGAIN:
            return false;

          case EIO:
            /* Could ignore EIO, see spec. */

            /* fall through */

          default:
            errno_msg("VIDIOC_DQBUF");
            exit(EXIT_FAILURE);
        }
      }

      assert (buf.index < n_buffers);

      process_image(v4l2frame, buffers[buf.index].start);

      if (-1 == xioctl(fd, VIDIOC_QBUF, &buf)) {
        errno_msg("VIDIOC_QBUF");
        exit(EXIT_FAILURE);
      }

      break;

    case IO_METHOD_USERPTR:
      CLEAR (buf);

      buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_USERPTR;

      if (-1 == xioctl(fd, VIDIOC_DQBUF, &buf)) {
        switch (errno) {
          case EAGAIN:
            return false;

          case EIO:
            /* Could ignore EIO, see spec. */

            /* fall through */

          default:
            errno_msg("VIDIOC_DQBUF");
            exit(EXIT_FAILURE);
        }
      }

      for (i = 0; i < n_buffers; ++i)
        if (buf.m.userptr == (unsigned long) buffers[i].start
            && buf.length == buffers[i].length)
          break;

      assert (i < n_buffers);

      process_image(v4l2frame, (void *) buf.m.userptr);

      if (-1 == xioctl(fd, VIDIOC_QBUF, &buf)) {
        errno_msg("VIDIOC_QBUF");
        exit(EXIT_FAILURE);
      }

      break;
  }

  return true;
}

void V4L2Helper::process_image(cv::Mat v4l2frame, const void *p) {
  int j = 0;
  char *tmp = (char *) p;
  char u, y0, y1, v;

  for (int i = 0; i < (width * height * 2); i += 4) {

    y0 = tmp[i];
    u = tmp[i + 1];
    y1 = tmp[i + 2];
    v = tmp[i + 3];

    v4l2frame.data[j++] = y0;
    v4l2frame.data[j++] = v;
    v4l2frame.data[j++] = u;

    v4l2frame.data[j++] = y1;
    v4l2frame.data[j++] = v;
    v4l2frame.data[j++] = u;
  }
}

//Must pass in IplImage* with pre-allocated imgData!
//IplImage* V4L2Helper::get_frame ()
void V4L2Helper::get_frame(cv::Mat frameToFill) {
  //IplImage *v4l2frame = cvCreateImage (cvSize (width, height), IPL_DEPTH_8U, 3);

  for (;;) {
    fd_set fds;
    struct timeval tv;
    int r;

    FD_ZERO (&fds);
    FD_SET (fd, &fds);

    /* Timeout. */
    tv.tv_sec = 2;
    tv.tv_usec = 0;

    r = select(fd + 1, &fds, NULL, NULL, &tv);

    if (-1 == r) {
      if (EINTR == errno)
        continue;

      errno_msg("select");
      //cvReleaseImage (&v4l2frame);
      exit(EXIT_FAILURE);
    }

    if (0 == r) {
      fprintf(stderr, "select timeout.\n");
      //cvReleaseImage (&v4l2frame);
      exit(EXIT_FAILURE);
    }

    if (read_frame(frameToFill))
      break;

    /* EAGAIN - continue select loop. */

  }

  //return v4l2frame;
}
/*
int main() 
{
  V4L2Helper v;
  v.init_V4L2Device ();
  IplImage *frame;
  for(;;) {
    frame = v.get_frame();
    cvReleaseImage(&frame);
  }
  return 0;
}

*/
