/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#include "FireWireHelpers.hpp"

FireWireHelper::FireWireHelper () {}

void FireWireHelper::clearSingle () {

  printf ("Clearing...\n");

  /* Stop transmission */
  dc1394_video_set_transmission (camera, DC1394_OFF);
  
  /* Stop capture */  
  dc1394_capture_stop (camera);  

  /* cleanup and exit */
  dc1394_camera_free (camera);

}

void FireWireHelper::clearMultiple () {

  printf ("Clearing multiple...\n");

  for (int i = 0; i < numCameras; i++) {

    /* Stop transmission */
    dc1394_video_set_transmission (cameras[i], DC1394_OFF);

    /* Stop capture */  
    dc1394_capture_stop (cameras[i]);

    /* cleanup and exit */
    dc1394_camera_free (cameras[i]);
  }
}

bool FireWireHelper::initializeFWSingle (int option, CAM_TYPE currentCam) {
 
  /* Initiazlie libdc1394 */
  d = dc1394_new ();  
  
  /* Find cameras */
  if (dc1394_camera_enumerate (d, &list) != DC1394_SUCCESS) {
    
    dc1394_log_error ("Enumeration failed.");
    clearSingle();
    return false;

  } else
    printf ("Enumeration successful.\n"); 
  
  /* Verify that we have at least one camera */  
  if (list->num == 0) {

    dc1394_log_error ("No cameras found.");
    clearSingle();
    return false;
    
  } else
    printf ("%d camera(s) found.\n", list->num);

  /* Always use camera with index 0 */
  camera = dc1394_camera_new (d, list->ids[0].guid);
  
  if (!camera) {

    dc1394_log_error ("Failed to initialize camera with guid %llx", list->ids[0].guid);
    clearSingle();
    return false;
    
  } else
    printf ("Initialized camera with GUID: %d.\n", list->ids[0].guid);
  
  dc1394_camera_free_list (list);

  // load default settings
  dc1394_memory_load (camera, 0);


  /* Set ISO speed */
  if (dc1394_video_set_iso_speed (camera, DC1394_ISO_SPEED_400) != DC1394_SUCCESS) {
    
    dc1394_log_error ("Could not set ISO speed.");
    clearSingle();
    return false;
    
  }

  if (option == STEREO_SYNC_CAM) {
    dc1394_video_set_mode (camera, DC1394_VIDEO_MODE_FORMAT7_3);
    
    err = dc1394_format7_set_roi (camera,
				  DC1394_VIDEO_MODE_FORMAT7_3,
				  DC1394_COLOR_CODING_RAW16,
				  // bytes per packet - sets frame rate
				  DC1394_USE_MAX_AVAIL,
				  0,
				  0,
				  640,
				  480);
    printf ("Using Format7 mode.\n");
  } else if (currentCam == BUMBLEBEE_CAM) {
    dc1394_video_set_mode(camera, DC1394_VIDEO_MODE_640x480_YUV422);
    printf ("Using 640x480 YUV422 mode.\n");
  } else if (currentCam == UNIBRAIN_CAM) {
    dc1394_video_set_mode(camera, DC1394_VIDEO_MODE_320x240_YUV422);
    printf ("Using 320x240 YUV422 mode.\n");
  } else {
    printf ("Error: Unable to assign video mode.\n");
    exit(0);
  }
  
  
  /* Set framerate */
  if (dc1394_video_set_framerate (camera, DC1394_FRAMERATE_30) != DC1394_SUCCESS) {
    
    dc1394_log_error ("Could not set framerate.");
    clearSingle();
    return false;

  }
  
  /* Setup capture */
  if (dc1394_capture_setup (camera, DC1394_ISO_SPEED_400, DC1394_CAPTURE_FLAGS_DEFAULT) != DC1394_SUCCESS) {
    
    dc1394_log_error ("Setup capture failed.");
    clearSingle();
    return false;
    
  } else
    printf ("Setup capture successful.\n");
  
  /* Start transmission */
  if (dc1394_video_set_transmission (camera, DC1394_ON) != DC1394_SUCCESS) {
    
    dc1394_log_error ("Failed to initialize transmission.");
    clearSingle();
    return false;

  } else    
    printf ("Transmission initialized.\n");  
  
  return true;
}

bool FireWireHelper::initializeFWMultiple (int n) {

  numCameras = n;
  
  /* Initiazlie libdc1394 */
  d = dc1394_new ();
  
  /* Find cameras */
  if (dc1394_camera_enumerate (d, &list) != DC1394_SUCCESS) {
    
    dc1394_log_error ("Enumeration failed.");
    return false;

  } else
    printf ("Enumeration successful.\n");

  /* Verify that we have specified number of cameras */

  if (list->num == 0) {

    dc1394_log_error ("No cameras found.");
    return false;

  } else
    printf ("%d cameras found.\n", list->num);


  for (int i = 0; i < numCameras; i++) {
      cameras[i] = dc1394_camera_new (d, list->ids[i].guid);
      if (!cameras[i]) {
	
	dc1394_log_error ("Failed to initialize camera %d with guid %llx", i, list->ids[i].guid);
	return false;
	
      } else
	printf ("Initialized camera with GUID: %d.\n", list->ids[i].guid);      
  }

  dc1394_camera_free_list (list);
  
  for (int i = 0; i < numCameras; i++) {

    /* Set ISO speed */
    if (dc1394_video_set_iso_speed (cameras[i], DC1394_ISO_SPEED_400) != DC1394_SUCCESS) {
      
      dc1394_log_error ("Could not set ISO speed for camera %d.", i);
      return false;
      
    }
    
    /* Set framerate */
    if (dc1394_video_set_framerate (cameras[i], DC1394_FRAMERATE_30) != DC1394_SUCCESS) {
      
      dc1394_log_error ("Could not set framerate for camera %d.", i);
      return false;
      
    }

    /* Setup capture */
    if (dc1394_capture_setup (cameras[i], DC1394_ISO_SPEED_400, DC1394_CAPTURE_FLAGS_DEFAULT) != DC1394_SUCCESS) {
      
      dc1394_log_error ("Setup capture failed for camera %d.", i);
      return false;
      
    } else
      printf ("Setup capture successful.\n");
    
  }

  for (int i = 0; i < numCameras; i++) {
    
    /* Start transmission */
    if (dc1394_video_set_transmission (cameras[i], DC1394_ON) != DC1394_SUCCESS) {
      
      dc1394_log_error ("Failed to initialize transmission for camera %d.", i);
      return false;
      
    } else    
      printf ("Transmission initialized.\n");  
    
  }
  
  return true;
}

void FireWireHelper::getStereoSyncFWFrames (cv::Mat sync_f1, cv::Mat sync_f2, cv::Mat disparity_src) {
  
  if (dc1394_capture_dequeue (camera, DC1394_CAPTURE_POLICY_WAIT, &frame) != DC1394_SUCCESS)
    dc1394_log_error ("Capture frame failed.");
  
  else {    

    // width and height of captured image
    int width = 640;
    int height = 480;   

    unsigned int combHeight = height*2;
    unsigned int bufSize = width*combHeight;
    
    unsigned char* src = frame->image;
    unsigned char* dst = (unsigned char*) malloc(bufSize);
    unsigned char* combRGB = (unsigned char*) malloc (3*bufSize);
    
    // Stereo images are interlaced in bayer format (raw16)
    dc1394_deinterlace_stereo (src, dst, width, combHeight);
    
    // Convert into RGB
    dc1394_bayer_decoding_8bit(dst, combRGB, width, combHeight, DC1394_COLOR_FILTER_BGGR, DC1394_BAYER_METHOD_SIMPLE);

    int j = 0;
    char u, y0, y1, v;
    cv::Mat temp(width, height, CV_8UC3);
    for (int i = 0; i < (width*height*2); i+=4){

      u = src[i];
      y0 = src[i+1];
      v = src[i+2];
      y1 = src[i+3];
      
      temp.data[j++] = y0;
      temp.data[j++] = v;
      temp.data[j++] = u;
      
      temp.data[j++] = y1;
      temp.data[j++] = v;
      temp.data[j++] = u;
    }
    temp.copyTo(disparity_src);
    
    // Save Left RGB Image
    unsigned char* RGBimg_L = combRGB + 3*width*height;        
    FILE* img_L = fopen("tmp/RGBimg_L.pgm", "wb");    
    fprintf (img_L, "P6\n%u %u\n255\n", width, height);
    fwrite ((const char *) RGBimg_L, 1, height*width*3, img_L);
    fclose (img_L);

    // Save Right RGB Image
    unsigned char* RGBimg_R = combRGB;
    FILE* img_R = fopen("tmp/RGBimg_R.pgm", "wb");    
    fprintf (img_R, "P6\n%u %u\n255\n", width, height);
    fwrite ((const char *) RGBimg_R, 1, height*width*3, img_R);
    fclose (img_R);
    
    sync_f1 = cv::imread("tmp/RGBimg_L.pgm", 1);
    sync_f2 = cv::imread("tmp/RGBimg_R.pgm", 1);
    
    dc1394_capture_enqueue (camera, frame);
    
    free(dst);
    free(combRGB);
  }
}


cv::Mat FireWireHelper::getSingleFWFrame (int index, CAM_TYPE currentCam) {

  if (index == MONO_CAM) {

    if (dc1394_capture_dequeue (camera, DC1394_CAPTURE_POLICY_WAIT, &frame) != DC1394_SUCCESS) {
      
      dc1394_log_error ("Capture frame failed.");
      return cv::Mat();
      
    } else {
      
      unsigned int width, height;
      if (currentCam == BUMBLEBEE_CAM) {
	dc1394_get_image_size_from_video_mode (camera, DC1394_VIDEO_MODE_640x480_YUV422, &width, &height);
	
      } else if (currentCam == UNIBRAIN_CAM) {
	dc1394_get_image_size_from_video_mode (camera, DC1394_VIDEO_MODE_320x240_YUV422, &width, &height);
	
      } else 
	printf ("Error: Unable to grab image size\n");
      
      cv::Mat fwframe(width, height, CV_8UC3);
      
      int j = 0;
      char *temp = (char*) frame->image;
      char u, y0, y1, v;
      
      for (int i = 0; i < (width*height*2); i+=4){
        u = temp[i];
        y0 = temp[i+1];
        v = temp[i+2];
        y1 = temp[i+3];

        fwframe.data[j++] = y0;
        fwframe.data[j++] = v;
        fwframe.data[j++] = u;

        fwframe.data[j++] = y1;
        fwframe.data[j++] = v;
        fwframe.data[j++] = u;
      }

      dc1394_capture_enqueue (camera, frame);

      /*      FILE* imagefile = fopen("tmp/gray.pgm", "wb");

      if (imagefile == NULL) {
	perror ("Can't create 'tmp/gray.pgm'");	
      }
      
      fprintf (imagefile, "P5\n%u %u \n255\n", width, height);
      fwrite (frame->image, 1, width*height, imagefile);
      fclose (imagefile);
      */
      return fwframe;
    }
  } else {

    if (dc1394_capture_dequeue (cameras[index], DC1394_CAPTURE_POLICY_WAIT, &frames[index]) != DC1394_SUCCESS) {
      dc1394_log_error ("Camera %d: Capture frame failed.", index);
      return cv::Mat();

    } else {

      unsigned int width, height;
      if (currentCam == BUMBLEBEE_CAM) {
	dc1394_get_image_size_from_video_mode (cameras[index], DC1394_VIDEO_MODE_640x480_YUV422, &width, &height);
	
      } else if (currentCam == UNIBRAIN_CAM) {
	dc1394_get_image_size_from_video_mode (cameras[index], DC1394_VIDEO_MODE_320x240_YUV422, &width, &height);
	
      } else 
	printf ("Error: Unable to grab image size\n");
      
      cv::Mat fwframe(width, height, CV_8UC3);
      
      int j = 0;
      
      char *temp = (char*) frames[index]->image;
      char u, y0, y1, v;
      
      for (int i = 0; i < (width*height*2); i+=4) {
        u = temp[i];
        y0 = temp[i+1];
        v = temp[i+2];
        y1 = temp[i+3];

        fwframe.data[j++] = y0;
        fwframe.data[j++] = v;
        fwframe.data[j++] = u;

        fwframe.data[j++] = y1;
        fwframe.data[j++] = v;
        fwframe.data[j++] = u;
      }
      
      dc1394_capture_enqueue (cameras[index], frames[index]);
      return fwframe;
    }    
  }
}

cv::Size FireWireHelper::getDimensions () {
  unsigned int width, height;
  dc1394_get_image_size_from_video_mode (camera, DC1394_VIDEO_MODE_640x480_YUV422, &width, &height);

  return cv::Size(width, height);
}


/*
  FireWireHelper fw;
  
int main()
{
  printf ("Initializing...\n");
  bool test;
  test = fw.initializeFWSingle ();
  printf ("True..? %d\n", test);
  fw.clearSingle ();
}
*/
