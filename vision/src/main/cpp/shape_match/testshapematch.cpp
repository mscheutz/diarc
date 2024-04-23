/**
 * Test Yefeng Zheng's (University of Maryland, College Park) shape context.
 * @author Michael Zillich
 * @date December 2013
 */

#include <stdio.h>
#include <shapecontext.h>

using namespace cv;
using namespace std;

int edgeThresh = 90;
Mat modelImage, modelGray, modelEdge, modelCedge;
Mat testImage, testGray, testEdge, testCedge;
Mat dbgImage;

// define a trackbar callback
static void onTrackbar(int, void*)
{
    blur(modelGray, modelEdge, Size(3,3));
    Canny(modelEdge, modelEdge, edgeThresh, edgeThresh*3, 3);
    modelCedge = Scalar::all(0);
    modelImage.copyTo(modelCedge, modelEdge);

    blur(testGray, testEdge, Size(3,3));
    Canny(testEdge, testEdge, edgeThresh, edgeThresh*3, 3);
    testCedge = Scalar::all(0);
    testImage.copyTo(testCedge, testEdge);

    printf("matching ...\n");
    matchEdgeImages(modelEdge, testEdge);
    printf("matching ... done\n");
    imshow("model", modelCedge);
    imshow("test", testCedge);
}

static void help()
{
    printf("\nTest Yefeng Zheng's shape congtext\n"
           "Call:\n"
           "    /.shapecontext model_image test_image\n");
}

const char* keys =
{
    "{1| |fruits.jpg|model image name}"
    "{2| |fruits.jpg|test image name}"
};

int main( int argc, const char** argv )
{
    help();

    CommandLineParser parser(argc, argv, keys);

    string filename = parser.get<string>("1");
    modelImage = imread(filename, 1);
    if(modelImage.empty())
    {
        printf("Cannot read image file: %s\n", filename.c_str());
        help();
        return -1;
    }
    filename = parser.get<string>("2");
    testImage = imread(filename, 1);
    if(testImage.empty())
    {
        printf("Cannot read image file: %s\n", filename.c_str());
        help();
        return -1;
    }

    modelCedge.create(modelImage.size(), modelImage.type());
    cvtColor(modelImage, modelGray, CV_BGR2GRAY);

    testCedge.create(testImage.size(), testImage.type());
    cvtColor(testImage, testGray, CV_BGR2GRAY);

    // Create a window
    namedWindow("model", 1);
    namedWindow("test", 1);

    // create a toolbar
    createTrackbar("Canny threshold", "model", &edgeThresh, 100, onTrackbar);

    // Show the image
    onTrackbar(0, 0);

    // Wait for a key stroke; the same function arranges events processing
    waitKey(0);

    return 0;
}
