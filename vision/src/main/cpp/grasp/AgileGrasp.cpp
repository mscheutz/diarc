/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

//
// Created by evan on 7/13/20.
//

#include "AgileGrasp.hpp"

#include "point_clouds/PCLFunctions.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/core/eigen.hpp>
#include <boost/thread.hpp>

//includes for parsing XML
#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/foreach.hpp>

using namespace diarc::grasp;

AgileGrasp::AgileGrasp() {
  logger = log4cxx::Logger::getLogger("diarc.detector.grasp.AgileGrasp");
}

AgileGrasp::~AgileGrasp() {

}

std::vector<Grasp> AgileGrasp::calculateGraspPoses(pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud, const cv::Mat &transform) {
  pcl::PointCloud<PointType>::Ptr object_cloud(new pcl::PointCloud<PointType>);
  pcl::copyPointCloud(*cloud, *object_cloud);

  //get camera Transform Base of the robot
  Eigen::Matrix4d cam_tf; //Matrix4d <double,4,4>
  cv2eigen(transform, cam_tf);

  //cam_tf appears twice since the original method uses two cameras. We only use one, thus, the same camera is the input for the two options (ONLY ONE PERSPECTIVE)
  localization->setCameraTransforms(cam_tf, cam_tf); //sets the camera transformation every iteration since it is not fixed

  //this vector needs to be empty, just a debugging input for localizeHands(), in case of debugging uncomment and change the values as desired
  std::vector<int> indices;

  std::vector<Handle> handles;
  {
    // pcl::ScopeTime t("Grasp Point Algorithm Time.");

    std::vector<GraspHypothesis> hands = localization->localizeHands(object_cloud, (int) object_cloud->size(), indices, antipodal, clustering_pc);
    LOG4CXX_DEBUG(logger, "[LOCALIZE HANDS DONE]");
    std::vector<GraspHypothesis> antipodal_hands = localization->predictAntipodalHands(hands, svm_file);
    LOG4CXX_DEBUG(logger, "[ANTIPODALS DONE]");
    handles = localization->findHandles(antipodal_hands, min_inliers, handle_min_length);
    LOG4CXX_DEBUG(logger, "[HANDLES DONE]");
  }

  //objects to collect the quaternion and points for all grasp info that will populate grasps
  pcl::PointCloud<pcl::PointXYZ>::Ptr points;
  Eigen::Quaternionf orientation;
  std::vector<Handle>::const_iterator handle_itr;

  // if there are handles (grasps) found
  std::vector<Grasp> grasps;
  if (handles.size() > 0) {
    for (handle_itr = handles.begin(); handle_itr != handles.end(); ++handle_itr) {
      // reset reused data structures
      points.reset(new pcl::PointCloud<pcl::PointXYZ>());

      // calculate the grasp
      calculateGraspPose(*handle_itr, points, orientation);

      // add grasp to results
      grasps.push_back(Grasp(points, orientation));
    }

    //plot coordinate frames for debugging purposes
    if (pcl_viz) { //to turn on/off, make changes in config XML
      pcl::PointCloud<PointType>::Ptr object_cloud_display(new pcl::PointCloud<PointType>);
      pcl::copyPointCloud(*cloud, *object_cloud_display);
      plotAntipodalFrames(handles, object_cloud_display);
    }
  }

  return grasps;
}

void AgileGrasp::calculateGraspPose(const Handle& antipodal, pcl::PointCloud<pcl::PointXYZ>::Ptr& points, Eigen::Quaternionf & quat) {

  //compute the quaternion 
  //surface point on the object into a Transformation Matrix (ONLY WITH TRANSLATION)
  Eigen::Vector3f grasp_surface = antipodal.getHandsCenter().cast<float>();
  Eigen::Affine3f t(Eigen::Translation3f(grasp_surface(0), grasp_surface(1), grasp_surface(2))); //x,y,z
  //transform quaternion of the rotation of the frame on the previous surface point into a Transformation Matrix (ONLY WITH ROTATION)
  Eigen::Affine3f q(calculateEndEffectorOrientation(antipodal));
  //Eigen::Quaternionf q = calculateEndEffectorOrientation(antipodal);
  //Multiply both Transformation Matrices to get one frame with the ROTATION AND TRANSLATION 
  Eigen::Affine3f transform = (t * q);

  //rotate the coordinate frame by y-90 and then z180 to achieve the desired pose for PR2 
  Eigen::Affine3f Y = Eigen::Affine3f(Eigen::AngleAxisf(-0.5 * M_PI, Eigen::Vector3f::UnitY()));
  Eigen::Affine3f Z = Eigen::Affine3f(Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitZ()));
  //Eigen::Affine3f X  = Eigen::Affine3f(Eigen::AngleAxisf(M_PI,Eigen::Vector3f::UnitX()));
  Eigen::Affine3f RXYZ = transform * Y * Z;

  //quaternion for orientation for PR2
  Eigen::Matrix4f pose_gripper = RXYZ.matrix();
  //extract the rotation matrix from the homogeneous transformation
  Eigen::Matrix3f rotation = pose_gripper.block(0, 0, 3, 3);
  quat = Eigen::Quaternionf(rotation);
  quat.normalize();

  //quaternion for PR2
  LOG4CXX_DEBUG(logger, boost::format("[QUATERNION PR2 x,y,z,w] [ %.3f, %.3f, %.3f , %.3f] ") % quat.x() % quat.y() % quat.z() % quat.w());

  //calculate points
  Eigen::Vector4f binormal = (antipodal.getWidth() / 2) * pose_gripper.col(1); //binormal * 0.5width_of_object

  //first point for grasp
  pcl::PointXYZ point;
  point.x = grasp_surface(0) - binormal(0);
  point.y = grasp_surface(1) - binormal(1);
  point.z = grasp_surface(2) - binormal(2);
  points->push_back(point);

  //second point 
  point.x = grasp_surface(0) + binormal(0);
  point.y = grasp_surface(1) + binormal(1);
  point.z = grasp_surface(2) + binormal(2);
  points->push_back(point);
}

Eigen::Quaternionf AgileGrasp::calculateEndEffectorOrientation(const Handle & antipodal) {
  Eigen::Matrix3f R = Eigen::MatrixXf::Zero(3, 3);
  //column three is the opposite direction approach vector which is orthogonal to axis vector
  R.col(2) = -1 * antipodal.getApproach().cast<float>();
  //column one is the axis of the object which is orthogonal to the approach vector
  R.col(0) = antipodal.getAxis().cast<float>();
  //the cross product of the two previous columns gives an orthogonal vector to complete the transformation matrix
  R.col(1) << R.col(2).cross(R.col(0));

  //convert the Rotation Matrix 'R' to Quaternion
  Eigen::Quaternionf quatf(R);
  //normalize the quaternion
  quatf.normalize();

  //quaternion from agile grasp
  //  LOG4CXX_DEBUG(logger, boost::format("[QUATERNION x,y,z,w] [ %.3f, %.3f, %.3f , %.3f] ") % quatf.x() % quatf.y() % quatf.z() % quatf.w());

  return quatf;
}

void AgileGrasp::plotAntipodalFrames(const std::vector<Handle>& antipodals, pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud) {

  //create the pcl viewer  
  std::string title = "Grasp Frames RED(x):axis GREEN(y):binormal  BLUE(z):approach";
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer(title));
  viewer->setBackgroundColor(1, 1, 1);
  //add cloud to viewer
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "object_cloud");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "object_cloud");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 0, "object_cloud");

  for (int i = 0; i < antipodals.size(); i++) {
    //surface point on the object into a Transformation Matrix (ONLY WITH TRANSLATION)
    Eigen::Vector3f grasp_surface = antipodals[i].getHandsCenter().cast<float>();
    Eigen::Affine3f t(Eigen::Translation3f(grasp_surface(0), grasp_surface(1), grasp_surface(2))); //x,y,z
    //transform quaternion of the rotation of the frame on the previous surface point into a Transformation Matrix (ONLY WITH ROTATION)
    Eigen::Affine3f q(calculateEndEffectorOrientation(antipodals[i]));
    //Multiply both Transformation Matrices to get one frame with the ROTATION AND TRANSLATION 
    Eigen::Affine3f transform = (t * q);
    //adds/attaches a coordinate system frame to the cloud
    viewer->addCoordinateSystem(0.03, transform); //0.03


    //rotate the coordinate frame by z-180 and then y90 to achieve the original orientation from agile_grasp quaternion 
    Eigen::Affine3f Y = Eigen::Affine3f(Eigen::AngleAxisf(0.5 * M_PI, Eigen::Vector3f::UnitY()));
    Eigen::Affine3f Z = Eigen::Affine3f(Eigen::AngleAxisf(-M_PI, Eigen::Vector3f::UnitZ()));
    //Eigen::Affine3f X  = Eigen::Affine3f(Eigen::AngleAxisf(M_PI,Eigen::Vector3f::UnitX()));
    Eigen::Affine3f RXYZ = transform * Z*Y;
    //viewer->addCoordinateSystem(0.07, RXYZ);

    //add a line for the points of PINCH
    pcl::PointXYZ p1, p2;
    Eigen::Matrix4f f = RXYZ.matrix();
    Eigen::Vector4f b = (antipodals[i].getWidth() / 2) * f.col(1); //binormal*1/2
    Eigen::Vector4f c = (antipodals[i].getWidth() / 2) * f.col(3);
    //Eigen::Vector3f b = (antipodals[i].getWidth()/2) *antipodals[i].getCenter().cast<float>();
    //p2.x = grasp_surface(0)-2*b(0);
    //p2.y = grasp_surface(1)-2*b(1);
    //p2.z = grasp_surface(2)-2*b(2);
    p2.x = grasp_surface(0) - b(0);
    p2.y = grasp_surface(1) - b(1);
    p2.z = grasp_surface(2) - b(2);

    //p1.x = grasp_surface(0)+2*b(0);
    //p1.y = grasp_surface(1)+2*b(1);
    //p1.z = grasp_surface(2)+2*b(2);
    p1.x = grasp_surface(0) + b(0);
    p1.y = grasp_surface(1) + b(1);
    p1.z = grasp_surface(2) + b(2);

    //viewer->addLine(p1, p2, 0.8, 0.7, 0.6, "line", 0);
    //viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 6, "line");
  }

  //runviewer until closed
  viewer->initCameraParameters();
  viewer->setPosition(0, 0);
  viewer->setSize(640, 480);

  while (!viewer->wasStopped()) {
    viewer->spinOnce(100);
    boost::this_thread::sleep(boost::posix_time::microseconds(100000));
  }

  viewer->close();
}


void AgileGrasp::loadConfig(const std::string &config) {
  LOG4CXX_DEBUG(logger, boost::format("[loadConfig] configFileName: %s.") % config);

  //configuration
  int num_samples;
  int num_threads;
  double taubin_radius;
  double hand_radius;
  double finger_width;
  double hand_outer_diameter;
  double hand_depth;
  double hand_height;
  double initial_bite;
  //double handle_min_length;

  //workspace dimensions (MODIFY IN XML)
  Eigen::VectorXd workspace(6);

  //XML steps to read it
  unsigned position = config.find_last_of("/\\");
  std::string dir = config.substr(0, position + 1);

  // populate tree structure pt
  using boost::property_tree::ptree;
  ptree pt;
  read_xml(config, pt);


  //get svm_file and data in XML parameters/nodes

  svm_file = "";
  BOOST_FOREACH(ptree::value_type const& dataNode, pt.get_child("AgileGrasp")) {
          if (dataNode.first.compare("svm_file") == 0) {
            std::string file = static_cast<std::string> (dataNode.second.data());
            LOG4CXX_DEBUG(logger, boost::format("[loadCOnfig] dir: %s. file: %s.") % dir % file);
            svm_file = dir + file;
          } else if (dataNode.first.compare("workspace") == 0) {
            workspace(0) = dataNode.second.get<int>("n_x");
            workspace(1) = dataNode.second.get<int>("x");
            workspace(2) = dataNode.second.get<int>("n_y");
            workspace(3) = dataNode.second.get<int>("y");
            workspace(4) = dataNode.second.get<int>("n_z");
            workspace(5) = dataNode.second.get<int>("z");
            LOG4CXX_DEBUG(logger, boost::format("[loadCOnfig] Workspace: %d, %d, %d, %d, %d, %d.") % workspace(0) % workspace(1) % workspace(2) % workspace(3) % workspace(4) % workspace(5));
          } else if (dataNode.first.compare("configuration") == 0) {
            num_samples = dataNode.second.get<int>("samples");
            num_threads = dataNode.second.get<int>("threads");
            min_inliers = dataNode.second.get<int>("min_inliers");
            taubin_radius = dataNode.second.get<double>("taubin_radius");
            hand_radius = dataNode.second.get<double>("hand_radius");
            finger_width = dataNode.second.get<double>("finger_width");
            hand_outer_diameter = dataNode.second.get<double>("hand_outer_diameter");
            hand_depth = dataNode.second.get<double>("hand_depth");
            hand_height = dataNode.second.get<double>("hand_height");
            initial_bite = dataNode.second.get<double>("initial_bite");
            handle_min_length = dataNode.second.get<double>("handle_min_length");
            LOG4CXX_DEBUG(logger, boost::format("[loadCOnfig] Samples: %d.") % num_samples);
            LOG4CXX_DEBUG(logger, boost::format("[loadCOnfig] Threads: %d.") % num_threads);
            LOG4CXX_DEBUG(logger, boost::format("[loadCOnfig] Inliers: %d.") % min_inliers);
            LOG4CXX_DEBUG(logger, boost::format("[loadCOnfig] Taubin radius: %f.") % taubin_radius);
            LOG4CXX_DEBUG(logger, boost::format("[loadCOnfig] Hand radius: %f.") % hand_radius);
            LOG4CXX_DEBUG(logger, boost::format("[loadCOnfig] Finger width: %f.") % finger_width);
            LOG4CXX_DEBUG(logger, boost::format("[loadCOnfig] Hand outer diameter: %f.") % hand_outer_diameter);
            LOG4CXX_DEBUG(logger, boost::format("[loadCOnfig] Hand depth: %f.") % hand_depth);
            LOG4CXX_DEBUG(logger, boost::format("[loadCOnfig] Hand height: %f.") % hand_height);
            LOG4CXX_DEBUG(logger, boost::format("[loadCOnfig] Initial bite: %f.") % initial_bite);
            LOG4CXX_DEBUG(logger, boost::format("[loadCOnfig] Handle Length: %f.") % handle_min_length);
          } else if (dataNode.first.compare("options") == 0) {
            plotting_mode = dataNode.second.get<int>("plot");
            pcl_viz = dataNode.second.get<bool>("pcl_viz");
            antipodal = dataNode.second.get<bool>("antipodal");
            clustering_pc = dataNode.second.get<bool>("clustering_pc");
            LOG4CXX_DEBUG(logger, boost::format("[loadCOnfig] Plot mode available (1-true): %d.") % plotting_mode);
            LOG4CXX_DEBUG(logger, boost::format("[loadCOnfig] Antipodal available: %f.") % antipodal);
            LOG4CXX_DEBUG(logger, boost::format("[loadCOnfig] Clustering available: %f.") % clustering_pc);
          }
        }

  //initialize Localization shared_Ptr
  localization = LocalizationPtr(new Localization(num_threads, true, plotting_mode));

  //set-up parameters for the hand search
  localization->setWorkspace(workspace);
  localization->setNumSamples(num_samples);
  localization->setNeighborhoodRadiusTaubin(taubin_radius);
  localization->setNeighborhoodRadiusHands(hand_radius);
  localization->setFingerWidth(finger_width);
  localization->setHandOuterDiameter(hand_outer_diameter);
  localization->setHandDepth(hand_depth);
  localization->setInitBite(initial_bite);
  localization->setHandHeight(hand_height);
}
