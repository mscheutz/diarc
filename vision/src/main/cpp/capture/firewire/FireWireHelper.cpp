/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#include "FireWireHelper.hpp"

/* Method calls related to firewire camera capture */

FireWireHelper::FireWireHelper()
: conversionImg(0) {
  cameras[0] = NULL;
  cameras[1] = NULL;
}

FireWireHelper::~FireWireHelper() {
    printf("[FireWireHelper] destructor\n");

    //release conversion image
    if (conversionImg) {
        cvReleaseImage(&conversionImg);
    }

    //cleanup cameras
    clearCameras();
    
    dc1394_free(d);
}

void FireWireHelper::resetBus() {
    dc1394_t *d;
    dc1394camera_list_t *list;
    dc1394camera_t * camera[MAX_CAMERAS];

    printf("\nAttempting to reset all cameras on firewire bus...\n");

    d = dc1394_new();
    if (dc1394_camera_enumerate(d, &list) != DC1394_SUCCESS) {
        dc1394_log_error("Enumeration failed.");
        return;
    } else {
        printf("Enumeration successful.\n");
    }

    if (list->num == 0) {
        dc1394_log_error("No cameras found");
        return;
    } else {
        printf("%d camera(s) found.\n", list->num);
    }

    /* Initialization loop has to be done separately, otherwise, only one will be initialized.*/
    for (int i = 0; i < list->num; i++) {
        camera[i] = dc1394_camera_new(d, list->ids[i].guid);
    }

    /* Reset cameras if they are initialized properly. */
    for (int i = 0; i < list->num; i++) {
        if (!camera[i]) {
            dc1394_log_error("Failed to initialize camera with GUID %d.", list->ids[i].guid);
        } else {
            printf("Resetting camera with GUID: %lu.\n", list->ids[i].guid);
            dc1394_video_set_transmission(camera[i], DC1394_OFF);
            dc1394_capture_stop(camera[i]);
            dc1394_reset_bus(camera[i]);
            dc1394_camera_free(camera[i]);
        }

    }

    dc1394_camera_free_list(list);
    dc1394_free(d);
    printf("Reset bus comeplete..!\n\n");

    return;
}

void FireWireHelper::clearCameras() {

    //printf("Clearing multiple : %d...\n", numCameras);

    for (int i = 0; i < numCameras; i++) {

        /* Stop transmission */
        dc1394_video_set_transmission(cameras[i], DC1394_OFF);

        /* Stop capture */
        dc1394_capture_stop(cameras[i]);

        /* cleanup and exit */
        dc1394_camera_free(cameras[i]);
    }
}

bool FireWireHelper::initializeFW(int numCams) {
    //resetBus();

    int err;
    uint32_t nCameras;

    /* Initialize libdc1394 */
    d = dc1394_new();

    /* Find cameras */
    err = dc1394_camera_enumerate(d, &list);
    if (err != DC1394_SUCCESS) {
        dc1394_log_error("Enumeration failed.");
        dc1394_camera_free_list(list);
        dc1394_free(d);
        return false;
    }

    /* Verify that we have enough cameras */
    nCameras = list->num;
    if (nCameras < numCams) {
        dc1394_log_error("Not enough cameras found.");
        dc1394_camera_free_list(list);
        dc1394_free(d);
        return false;
    }
    numCameras = numCams;
    printf("There were %d cameras found attached to your PC\n", nCameras);

    for (unsigned int i = 0; i < numCams; ++i) {
        cameras[i] = dc1394_camera_new(d, list->ids[i].guid);

        dc1394video_mode_t video_mode;
        dc1394color_coding_t coding;

        if ((int) list->ids[i].guid == UNIBRAIN_SN1) {
            //            printf("unibrain video mode\n");
            video_mode = DC1394_VIDEO_MODE_320x240_YUV422;
        } else {
            //For MDS Firefly Cameras
            video_mode = DC1394_VIDEO_MODE_640x480_MONO8;
            /*
                        // get the highest resolution video mode. This can be skipped
                        // if you already know which mode/framerate you want...
                        // note: you are getting a RGB8 mode, so onboard color processing is required
                        // For cameras without onboard color processing, see the grabbayer.cpp example
             */
            // get video modes:
            dc1394video_modes_t video_modes;
            if (dc1394_video_get_supported_modes(cameras[i], &video_modes) != DC1394_SUCCESS) {
                fprintf(stderr, "Can't get video modes\n");
                cleanup(cameras, i);
                return false;
            }

            for (int i = video_modes.num - 1; i >= 0; i--) {
                printf("video mode option %d: %d\n", i, video_modes.modes[i]);
            }
            /*
                        // select highest res mode that is RGB8
                        printf("Searching for the highest resolution RGB8 mode available...\n");
            
                        for (int i = video_modes.num - 1; i >= 0; i--) {
                            printf("video mode option %d: %d\n", i, video_modes.modes[i]);
                            // don't consider FORMAT 7 modes (i.e. "scalable")
                            if (!dc1394_is_video_mode_scalable(video_modes.modes[i])) {
                                dc1394_get_color_coding_from_video_mode(cameras[i], video_modes.modes[i], &coding);
                                if (coding == DC1394_COLOR_CODING_RGB8) {
                                    video_mode = video_modes.modes[i];
                                    break;
                                }
                            }
                        }

                        // double check that we found a video mode  that is RGB8
                        dc1394_get_color_coding_from_video_mode(cameras[i], video_mode, &coding);
                        if ((coding != DC1394_COLOR_CODING_RGB8)) {
                            fprintf(stderr, "Could not get a valid RGB8 mode\n");
                            cleanup(cameras, i);
                            return false;
                        }
             */
        }


        //#ifdef USE_1394B
        //       dc1394_video_set_operation_mode(cameras[i], DC1394_OPERATION_MODE_1394B);
        //      dc1394_video_set_iso_speed(cameras[i], DC1394_ISO_SPEED_800);
        //#else
        dc1394_video_set_iso_speed(cameras[i], DC1394_ISO_SPEED_400);
        //#endif

        /*
                // get highest framerate
                dc1394framerates_t framerates;
                dc1394framerate_t framerate;
                if (dc1394_video_get_supported_framerates(cameras[i], video_mode, &framerates) != DC1394_SUCCESS) {
                    fprintf(stderr, "Can't get a framerate\n");
            
                    cleanup(cameras, i);
                    return false;
                }
                framerate = framerates.framerates[ framerates.num - 1 ];
         */
        dc1394framerate_t framerate = DC1394_FRAMERATE_15;


        // store video_mode and color_coding for retrieving images
        modes[i] = video_mode;
        dc1394_get_color_coding_from_video_mode(cameras[i], video_mode, &coding);
        codings[i] = coding;

        // set video mode and framerate
        dc1394_video_set_mode(cameras[i], video_mode);
        dc1394_video_set_framerate(cameras[i], framerate);

        // setup capture
        printf("Setting capture\n");
        if (dc1394_capture_setup(cameras[i], DC1394_ISO_SPEED_400, DC1394_CAPTURE_FLAGS_DEFAULT) != DC1394_SUCCESS) {
            fprintf(stderr, "unable to setup camera-\n"
                    "check line %d of %s to make sure\n"
                    "that the video mode and framerate are\n"
                    "supported by your camera\n",
                    __LINE__, __FILE__);
            fprintf(stderr,
                    "video_mode = %d, framerate = %d\n"
                    "Check dc1394_control.h for the meanings of these values\n",
                    video_mode, framerate);

            cleanup(cameras, i);
            return false;
        }

        // have the camera start sending us data
        printf("Start transmission\n");
        if (dc1394_video_set_transmission(cameras[i], DC1394_ON) != DC1394_SUCCESS) {
            fprintf(stderr, "Unable to start camera iso transmission\n");
            cleanup(cameras, i);
            return false;
        }
    }

    dc1394_camera_free_list(list);
    return true;
}

void FireWireHelper::cleanup(dc1394camera_t** cameras, int num) {
    for (int i = 0; i < num; ++i) {
        dc1394_capture_stop(cameras[i]);
        dc1394_video_set_transmission(cameras[i], DC1394_OFF);
        dc1394_camera_free(cameras[i]);
    }
    dc1394_camera_free_list(list);
    dc1394_free(d);
}

//Must pass in IplImage with pre-allocated image data!!  Make sure you cvReleaseImage!
//IplImage* FireWireHelper::getSingleFWFrame (int index, CAM_TYPE currentCam)

//Returns RGB frame

bool FireWireHelper::getFWFrame(int index, IplImage* frameToFill) {
    //    printf("Capture dequeue\n");
    if (dc1394_capture_dequeue(cameras[index], DC1394_CAPTURE_POLICY_WAIT, &frames[index]) != DC1394_SUCCESS) {
        dc1394_log_error("Camera %d: Capture frame failed.", index);
        return false;

    } else {
        //        printf("get img size\n");
        unsigned int in_width, in_height;
        dc1394_get_image_size_from_video_mode(cameras[index], modes[index], &in_width, &in_height);

        unsigned int out_width = frameToFill->width;
        unsigned int out_height = frameToFill->height;

        char *temp = (char*) frames[index]->image;

        //allocate conversion image
        if (!conversionImg) {
            conversionImg = cvCreateImage(cvSize(in_width, in_height), IPL_DEPTH_8U, 3);
        }

        if (DC1394_COLOR_CODING_YUV422 == codings[index]) {
            int j = 0;
            char u, y0, y1, v;

            //            printf("convert image\n");
            //            printf("image size: %d x %d\n", in_width, in_height);
            for (int i = 0; i < (in_width * in_height * 2); i += 4) {

                u = temp[i];
                y0 = temp[i + 1];
                v = temp[i + 2];
                y1 = temp[i + 3];

                conversionImg->imageData[j++] = y0;
                conversionImg->imageData[j++] = v;
                conversionImg->imageData[j++] = u;

                conversionImg->imageData[j++] = y1;
                conversionImg->imageData[j++] = v;
                conversionImg->imageData[j++] = u;
            }

            //copy to output buffer, and resize (if needed)
            if (out_width != in_width || out_height != in_height) {
                cvResize(conversionImg, frameToFill);
            } else {
                cvCopy(conversionImg, frameToFill); //EP cvCopyImage -> cvCopy
            }

            //convert to RGB
            cvCvtColor(frameToFill, frameToFill, CV_YCrCb2BGR);

        } else if (DC1394_COLOR_CODING_RGB8 == codings[index]) {
            for (int i = 0; i < (in_width * in_height * 3); i += 3) {
                conversionImg->imageData[i] = temp[i];
                conversionImg->imageData[i + 1] = temp[i + 1];
                conversionImg->imageData[i + 2] = temp[i + 2];
            }

            if (out_width != in_width || out_height != in_height) {
                cvResize(conversionImg, frameToFill);
            } else {
                cvCopy(conversionImg, frameToFill); //EP cvCopyImage -> cvCopy
            }
        } else if (DC1394_COLOR_CODING_MONO8 == codings[index]) {
            /*
                            // get bayer tile coding
                            uint32_t value;
                            dc1394_get_control_register(cameras[index], 0x1040, &value);

            dc1394color_filter_t bayerPattern;// = DC1394_COLOR_FILTER_RGGB;
            switch( value )
               {
                  default:
                  case 0x59595959:	// YYYY
                     // no bayer
                     bayerPattern = (dc1394color_filter_t) 0;
                     printf("case 1\n");
                     break;
                  case 0x52474742:	// RGGB
                     bayerPattern = DC1394_COLOR_FILTER_RGGB;
                     printf("case 2\n");
                     break;
                  case 0x47425247:	// GBRG
                     bayerPattern = DC1394_COLOR_FILTER_GBRG;
                     printf("case 3\n");
                     break;
                  case 0x47524247:	// GRBG
                     bayerPattern = DC1394_COLOR_FILTER_GRBG;
                     printf("case 4\n");
                     break;
                  case 0x42474752:	// BGGR
                     bayerPattern = DC1394_COLOR_FILTER_BGGR;
                     printf("case 5\n");
                     break;
               }
             */

            // Convert into RGB
            dc1394color_filter_t bayerPattern = DC1394_COLOR_FILTER_RGGB;
            dc1394_bayer_decoding_8bit((uchar*) temp, (uchar*) conversionImg->imageData, in_width, in_height, bayerPattern, DC1394_BAYER_METHOD_NEAREST);


            if (out_width != in_width || out_height != in_height) {
                cvResize(conversionImg, frameToFill);
            } else {
                cvCopy(conversionImg, frameToFill); //EP cvCopyImage -> cvCopy
            }
        } else {
            printf("[FireWireHelper::getFWFrame]: Currently unsupported color conversion.");
            return false;
        }

        //        printf("Capture enqueue\n");
        dc1394_capture_enqueue(cameras[index], frames[index]);
    }
    
    return true;
}

bool FireWireHelper::initializeFWSingle(int option, CAM_TYPE currentCam) {
    numCameras = 1;
    
    /* Reset firewire bus first */
    resetBus();

    /* Initialize libdc1394 */
    d = dc1394_new();

    /* Find cameras */
    if (dc1394_camera_enumerate(d, &list) != DC1394_SUCCESS) {

        dc1394_log_error("Enumeration failed.");
        dc1394_camera_free_list(list);
        dc1394_free(d);
        return false;

    } else
        printf("Enumeration successful.\n");

    /* Verify that we have at least one camera */
    if (list->num == 0) {

        dc1394_log_error("No cameras found.");
        dc1394_camera_free_list(list);
        dc1394_free(d);
        return false;

    } else
        printf("%d camera(s) found.\n", list->num);

    int init_index;

    /* Search for bumblebee camera */
    if (currentCam == BUMBLEBEE_CAM) {
        for (int i = 0; i < list->num; i++) {
            if ((int) list->ids[i].guid == BUMBLEBEE_SN) {
                cameras[0] = dc1394_camera_new(d, list->ids[i].guid);
                init_index = i;
                break;
            }
        }
    }/* Search for unibrain camera
     (Note: For now, only one unibrain camera should be on the firewire bus,
     otherwise, frames may overlap between cameras.)
         */
    else {
        for (int i = 0; i < list->num; i++) {
            if ((int) list->ids[i].guid != BUMBLEBEE_SN) {
                cameras[0] = dc1394_camera_new(d, list->ids[i].guid);
                init_index = i;
                break;
            }
        }
    }

    if (!cameras[0]) {

        dc1394_log_error("Failed to initialize camera with GUID: %d.", list->ids[init_index].guid);
        clearCameras();
        dc1394_camera_free_list(list);
        dc1394_free(d);
        return false;

    } else
        printf("Initialized camera with GUID: %lu.\n", list->ids[init_index].guid);

    // load default settings
    dc1394_memory_load(cameras[0], 0);

    /* Set ISO speed */
    if (dc1394_video_set_iso_speed(cameras[0], DC1394_ISO_SPEED_400) != DC1394_SUCCESS) {

        dc1394_log_error("Could not set ISO speed.");
        clearCameras();
        dc1394_camera_free_list(list);
        dc1394_free(d);
        return false;

    }

    if (option == STEREO_SYNC_CAM) {
        dc1394_video_set_mode(cameras[0], DC1394_VIDEO_MODE_FORMAT7_3);

        err = dc1394_format7_set_roi(cameras[0],
                DC1394_VIDEO_MODE_FORMAT7_3,
                DC1394_COLOR_CODING_RAW16,
                // bytes per packet - sets frame rate
                DC1394_USE_MAX_AVAIL,
                0,
                0,
                640,
                480);
        /* quick fix (needs better solution) */
        if (err < 0) {
            cameras[0] = dc1394_camera_new(d, list->ids[1].guid);
            printf("Reinitialized camera with GUID: %lu.\n", list->ids[1].guid);
            err = dc1394_format7_set_roi(cameras[0],
                    DC1394_VIDEO_MODE_FORMAT7_3,
                    DC1394_COLOR_CODING_RAW16,
                    // bytes per packet - sets frame rate
                    DC1394_USE_MAX_AVAIL,
                    0,
                    0,
                    640,
                    480);
        }
        printf("Using Format7 mode.\n");
    } else if (currentCam == BUMBLEBEE_CAM) {
        dc1394_video_set_mode(cameras[0], DC1394_VIDEO_MODE_640x480_YUV422);
        printf("Using 640x480 YUV422 mode.\n");
    } else if (currentCam == UNIBRAIN_CAM) {
        dc1394_video_set_mode(cameras[0], DC1394_VIDEO_MODE_320x240_YUV422);
        printf("Using 320x240 YUV422 mode.\n");
    } else {
        printf("Error: Unable to assign video mode.\n");
        exit(0);
    }


    /* Set framerate */
    if (dc1394_video_set_framerate(cameras[0], DC1394_FRAMERATE_30) != DC1394_SUCCESS) {

        dc1394_log_error("Could not set framerate.");
        clearCameras();
        dc1394_camera_free_list(list);
        dc1394_free(d);
        return false;

    }

    /* Setup capture */
    if (dc1394_capture_setup(cameras[0], DC1394_ISO_SPEED_400, DC1394_CAPTURE_FLAGS_DEFAULT) != DC1394_SUCCESS) {

        dc1394_log_error("Setup capture failed.");
        clearCameras();
        dc1394_camera_free_list(list);
        dc1394_free(d);
        return false;

    } else
        printf("Setup capture successful.\n");

    /* Start transmission */
    if (dc1394_video_set_transmission(cameras[0], DC1394_ON) != DC1394_SUCCESS) {

        dc1394_log_error("Failed to initialize transmission.");
        clearCameras();
        dc1394_camera_free_list(list);
        dc1394_free(d);
        return false;

    } else
        printf("Transmission initialized.\n");

    dc1394_camera_free_list(list);
    return true;
}

bool FireWireHelper::initializeFWMultiple(int n) {
    numCameras = n;

    /* Reset firewire bus first */
    resetBus();

    /* Initiazlie libdc1394 */
    d = dc1394_new();

    /* Find cameras */
    if (dc1394_camera_enumerate(d, &list) != DC1394_SUCCESS) {

        dc1394_log_error("Enumeration failed.");
        return false;

    } else
        printf("Enumeration successful.\n");

    /* Verify that we have specified number of cameras */

    if (list->num == 0) {

        dc1394_log_error("No cameras found.");
        return false;

    } else
        printf("%d cameras found.\n", list->num);


    for (int i = 0; i < numCameras; i++) {
        cameras[i] = dc1394_camera_new(d, list->ids[i].guid);
        if (!cameras[i]) {

            dc1394_log_error("Failed to initialize camera %d with GUID: %d.", i, list->ids[i].guid);
            return false;

        } else
            printf("Initialized camera with GUID: %lu.\n", list->ids[i].guid);
    }

    dc1394_camera_free_list(list);

    for (int i = 0; i < numCameras; i++) {

        /* Set ISO speed */
        if (dc1394_video_set_iso_speed(cameras[i], DC1394_ISO_SPEED_400) != DC1394_SUCCESS) {

            dc1394_log_error("Could not set ISO speed for camera %d.", i);
            return false;

        }

        /* Set framerate */
        if (dc1394_video_set_framerate(cameras[i], DC1394_FRAMERATE_30) != DC1394_SUCCESS) {

            dc1394_log_error("Could not set framerate for camera %d.", i);
            return false;

        }

        /* Setup capture */
        if (dc1394_capture_setup(cameras[i], DC1394_ISO_SPEED_400, DC1394_CAPTURE_FLAGS_DEFAULT) != DC1394_SUCCESS) {

            dc1394_log_error("Setup capture failed for camera %d.", i);
            return false;

        } else
            printf("Setup capture successful.\n");

    }

    for (int i = 0; i < numCameras; i++) {

        /* Start transmission */
        if (dc1394_video_set_transmission(cameras[i], DC1394_ON) != DC1394_SUCCESS) {

            dc1394_log_error("Failed to initialize transmission for camera %d.", i);
            return false;

        } else
            printf("Transmission initialized.\n");

    }

    return true;
}

//EAK: This method is scary. First, it saves each image to file and then reloads
//them for some reason, and second, it allocated new memory for the passed in images
//so the caller needs to be sure to clean up the memory.

void FireWireHelper::getStereoSyncFWFrames(IplImage **sync_f1, IplImage **sync_f2, IplImage **disparity_src) {

    if (dc1394_capture_dequeue(cameras[0], DC1394_CAPTURE_POLICY_WAIT, &frames[0]) != DC1394_SUCCESS)
        dc1394_log_error("Capture frame failed.");

    else {

        // width and height of captured image
        int width = 640;
        int height = 480;

        unsigned int combHeight = height * 2;
        unsigned int bufSize = width*combHeight;

        unsigned char* src = frames[0]->image;
        unsigned char* dst = (unsigned char*) malloc(bufSize);
        unsigned char* combRGB = (unsigned char*) malloc(3 * bufSize);

        // Stereo images are interlaced in bayer format (raw16)
        dc1394_deinterlace_stereo(src, dst, width, combHeight);

        // Convert into RGB
        dc1394_bayer_decoding_8bit(dst, combRGB, width, combHeight, DC1394_COLOR_FILTER_BGGR, DC1394_BAYER_METHOD_SIMPLE);

        int j = 0;
        char u, y0, y1, v;
        IplImage *temp = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);
        for (int i = 0; i < (width * height * 2); i += 4) {

            u = src[i];
            y0 = src[i + 1];
            v = src[i + 2];
            y1 = src[i + 3];

            temp->imageData[j++] = y0;
            temp->imageData[j++] = v;
            temp->imageData[j++] = u;

            temp->imageData[j++] = y1;
            temp->imageData[j++] = v;
            temp->imageData[j++] = u;
        }

        *disparity_src = temp;

        // Save Left RGB Image

        unsigned char* RGBimg_L = combRGB + 3 * width*height;
        FILE* img_L = fopen("tmp/RGBimg_L.pgm", "wb");
        fprintf(img_L, "P6\n%u %u\n255\n", width, height);
        fwrite((const char *) RGBimg_L, 1, height * width * 3, img_L);
        fclose(img_L);

        // Save Right RGB Image
        unsigned char* RGBimg_R = combRGB;
        FILE* img_R = fopen("tmp/RGBimg_R.pgm", "wb");
        fprintf(img_R, "P6\n%u %u\n255\n", width, height);
        fwrite((const char *) RGBimg_R, 1, height * width * 3, img_R);
        fclose(img_R);

        *sync_f1 = cvLoadImage("tmp/RGBimg_L.pgm", 1);
        *sync_f2 = cvLoadImage("tmp/RGBimg_R.pgm", 1);

        dc1394_capture_enqueue(cameras[0], frames[0]);

        free(dst);
        free(combRGB);
    }
}

//Must pass in IplImage with pre-allocated image data!!  Make sure you cvReleaseImage!
//IplImage* FireWireHelper::getSingleFWFrame (int index, CAM_TYPE currentCam)

void FireWireHelper::getSingleFWFrame(int index, CAM_TYPE currentCam, IplImage* frameToFill) {
    if (index == MONO_CAM) {

        if (dc1394_capture_dequeue(cameras[0], DC1394_CAPTURE_POLICY_WAIT, &frames[0]) != DC1394_SUCCESS) {

            dc1394_log_error("Capture frame failed.");
            return;

        } else {

            unsigned int width, height;
            if (currentCam == BUMBLEBEE_CAM) {
                dc1394_get_image_size_from_video_mode(cameras[0], DC1394_VIDEO_MODE_640x480_YUV422, &width, &height);

            } else if (currentCam == UNIBRAIN_CAM) {
                dc1394_get_image_size_from_video_mode(cameras[0], DC1394_VIDEO_MODE_320x240_YUV422, &width, &height);

            } else
                printf("Error: Unable to grab image size\n");

            //IplImage* fwframe = cvCreateImage (cvSize (width, height), IPL_DEPTH_8U, 3);

            int j = 0;
            char *temp = (char*) frames[0]->image;
            char u, y0, y1, v;

            for (int i = 0; i < (width * height * 2); i += 4) {

                u = temp[i];
                y0 = temp[i + 1];
                v = temp[i + 2];
                y1 = temp[i + 3];

                frameToFill->imageData[j++] = y0;
                frameToFill->imageData[j++] = v;
                frameToFill->imageData[j++] = u;

                frameToFill->imageData[j++] = y1;
                frameToFill->imageData[j++] = v;
                frameToFill->imageData[j++] = u;
            }

            dc1394_capture_enqueue(cameras[0], frames[0]);

            /*      FILE* imagefile = fopen("tmp/gray.pgm", "wb");
			 
             if (imagefile == NULL) {
             perror ("Can't create 'tmp/gray.pgm'");
             }
			 
             fprintf (imagefile, "P5\n%u %u \n255\n", width, height);
             fwrite (frame->image, 1, width*height, imagefile);
             fclose (imagefile);
             */
            //return fwframe;
        }
    } else {

        if (dc1394_capture_dequeue(cameras[index], DC1394_CAPTURE_POLICY_WAIT, &frames[index]) != DC1394_SUCCESS) {
            dc1394_log_error("Camera %d: Capture frame failed.", index);
            return;

        } else {

            unsigned int width, height;
            if (currentCam == BUMBLEBEE_CAM) {
                dc1394_get_image_size_from_video_mode(cameras[index], DC1394_VIDEO_MODE_640x480_YUV422, &width, &height);

            } else if (currentCam == UNIBRAIN_CAM) {
                dc1394_get_image_size_from_video_mode(cameras[index], DC1394_VIDEO_MODE_320x240_YUV422, &width, &height);

            } else
                printf("Error: Unable to grab image size\n");

            //IplImage* fwframe = cvCreateImage (cvSize (width, height), IPL_DEPTH_8U, 3);

            int j = 0;

            char *temp = (char*) frames[index]->image;
            char u, y0, y1, v;

            for (int i = 0; i < (width * height * 2); i += 4) {

                u = temp[i];
                y0 = temp[i + 1];
                v = temp[i + 2];
                y1 = temp[i + 3];

                frameToFill->imageData[j++] = y0;
                frameToFill->imageData[j++] = v;
                frameToFill->imageData[j++] = u;

                frameToFill->imageData[j++] = y1;
                frameToFill->imageData[j++] = v;
                frameToFill->imageData[j++] = u;
            }

            dc1394_capture_enqueue(cameras[index], frames[index]);
            //return fwframe;
        }
    }
}

//Must pass in IplImage with pre-allocated image data!!  Make sure you cvReleaseImage!
//IplImage* FireWireHelper::getSingleRotatedFWFrame (double angle, int index, CAM_TYPE currentCam) {

void FireWireHelper::getSingleRotatedFWFrame(double angle, int index, CAM_TYPE currentCam, IplImage* frameToFill) {

    if (index == MONO_CAM) {

        if (dc1394_capture_dequeue(cameras[0], DC1394_CAPTURE_POLICY_WAIT, &frames[0]) != DC1394_SUCCESS) {

            fprintf(stderr, "Unable to capture a frame\n");
            return;

        } else {

            unsigned int width, height;
            if (currentCam == BUMBLEBEE_CAM) {
                dc1394_get_image_size_from_video_mode(cameras[0], DC1394_VIDEO_MODE_640x480_YUV422, &width, &height);

            } else if (currentCam == UNIBRAIN_CAM) {
                dc1394_get_image_size_from_video_mode(cameras[0], DC1394_VIDEO_MODE_320x240_YUV422, &width, &height);

            } else
                printf("Error: Unable to grab image size\n");

            //TODO: make this member variable to eliminate alloc/dealloc (ie. cvCreateImage/cvReleaseImage) every call??
            IplImage* fwframe = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);

            char* temp = (char*) frames[0]->image;
            char u, y0, y1, v;
            int j = 0;

            for (int i = 0; i < (width * height * 2); i += 4) {

                u = temp[i];
                y0 = temp[i + 1];
                v = temp[i + 2];
                y1 = temp[i + 3];

                fwframe->imageData[j++] = y0;
                fwframe->imageData[j++] = v;
                fwframe->imageData[j++] = u;

                fwframe->imageData[j++] = y1;
                fwframe->imageData[j++] = v;
                fwframe->imageData[j++] = u;
            }

            //IplImage* fwrotate = cvCreateImage (cvSize (height, width), IPL_DEPTH_8U, 3);

            float m[6];
            int w = width;
            int h = height;
            double a = angle * CV_PI / 180;

            CvMat M = cvMat(2, 3, CV_32F, m);

            m[0] = (float) cos(a);
            m[1] = (float) sin(a);
            m[2] = w * 0.5f;
            m[3] = -m[1];
            m[4] = m[0];
            m[5] = h * 0.5f;

            cvGetQuadrangleSubPix(fwframe, frameToFill, &M);

            cvReleaseImage(&fwframe);

            dc1394_capture_enqueue(cameras[0], frames[0]);
            //return fwrotate;
        }
    } else {
        if (dc1394_capture_dequeue(cameras[index], DC1394_CAPTURE_POLICY_WAIT, &frames[index]) != DC1394_SUCCESS) {

            fprintf(stderr, "Unable to capture a frame\n");
            return;

        } else {

            unsigned int width, height;
            if (currentCam == BUMBLEBEE_CAM) {
                dc1394_get_image_size_from_video_mode(cameras[index], DC1394_VIDEO_MODE_640x480_YUV422, &width, &height);

            } else if (currentCam == UNIBRAIN_CAM) {
                dc1394_get_image_size_from_video_mode(cameras[index], DC1394_VIDEO_MODE_320x240_YUV422, &width, &height);

            } else
                printf("Error: Unable to grab image size\n");

            //TODO: make this member variable to eliminate alloc/dealloc (ie. cvCreateImage/cvReleaseImage) every call
            IplImage* fwframe = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);

            char* temp = (char*) frames[index]->image;
            char u, y0, y1, v;
            int j = 0;

            for (int i = 0; i < (width * height * 2); i += 4) {

                u = temp[i];
                y0 = temp[i + 1];
                v = temp[i + 2];
                y1 = temp[i + 3];

                fwframe->imageData[j++] = y0;
                fwframe->imageData[j++] = v;
                fwframe->imageData[j++] = u;

                fwframe->imageData[j++] = y1;
                fwframe->imageData[j++] = v;
                fwframe->imageData[j++] = u;
            }

            //IplImage* fwrotate = cvCreateImage (cvSize (height, width), IPL_DEPTH_8U, 3);

            float m[6];
            int w = width;
            int h = height;
            double a = angle * CV_PI / 180;

            CvMat M = cvMat(2, 3, CV_32F, m);

            m[0] = (float) cos(a);
            m[1] = (float) sin(a);
            m[2] = w * 0.5f;
            m[3] = -m[1];
            m[4] = m[0];
            m[5] = h * 0.5f;

            cvGetQuadrangleSubPix(fwframe, frameToFill, &M);

            cvReleaseImage(&fwframe);

            dc1394_capture_enqueue(cameras[index], frames[index]);
            //return fwrotate;
        }
    }
}

CvSize FireWireHelper::getDimensions(int index) {
    unsigned int width = -1, height = -1;
    if (cameras[index]) {
        dc1394_get_image_size_from_video_mode(cameras[index], DC1394_VIDEO_MODE_640x480_YUV422, &width, &height);
    }

    return cvSize(width, height);
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
