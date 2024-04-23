/*
 Copyright © Thinking Robots, Inc., Tufts University, and others 2024.
 */

#include "Display.hpp"

namespace ade {

  //initalize static variables
  Display::Display2D_Map Display::displays2d;
  Display::Display3D_Map Display::displays3d;
  boost::mutex Display::display_mutex;

  log4cxx::LoggerPtr Display::logger = log4cxx::Logger::getLogger("ade.Display");

  Display::Display() {
    LOG4CXX_DEBUG(Display::logger, "[display] constructor.");
  }

  Display::~Display() {
  }

  void Display::run() {
    //scoped lock
    boost::lock_guard<boost::mutex> lock(display_mutex);

    //display all 2D windows
    Display2D_Map::const_iterator itr_2d;
    for (itr_2d = displays2d.begin(); itr_2d != displays2d.end(); ++itr_2d) {
      //cout << "name: " << name << " currWindowName: " << (*itr)->getName() << endl;
      (*itr_2d).second->display();
    }

    //display all 3D windows
    Display3D_Map::const_iterator itr_3d;
    for (itr_3d = displays3d.begin(); itr_3d != displays3d.end(); ++itr_3d) {
      //cout << "name: " << name << " currWindowName: " << (*itr)->getName() << endl;
      (*itr_3d).second->display();
    }

    //Must be called for opencv highgui windows to display properly
    if (cv::waitKey(2) >= 0); // cvShowImage() can not be called during cvWait()

  }

  void Display::displayFrame(const cv::Mat frameToDraw, const std::string& windowName) {
    //scoped lock
    boost::lock_guard<boost::mutex> lock(display_mutex);

    Display2D_Map::const_iterator itr_2d = displays2d.find(windowName);
    if (displays2d.end() != itr_2d) {
      (*itr_2d).second->setDisplayFrame(frameToDraw);
    }
  }

  void Display::displayPointCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, const std::string& cloudName, const std::string& windowName) {
    //scoped lock
    boost::lock_guard<boost::mutex> lock(display_mutex);

    Display3D_Map::const_iterator itr_3d = displays3d.find(windowName);
    if (displays3d.end() != itr_3d) {
      (*itr_3d).second->setPointCloud(cloud, cloudName);
    }
  }

  void Display::displayPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, const std::string& cloudName, const std::string& windowName) {
    //scoped lock
    boost::lock_guard<boost::mutex> lock(display_mutex);

    Display3D_Map::const_iterator itr_3d = displays3d.find(windowName);
    if (displays3d.end() != itr_3d) {
      (*itr_3d).second->setPointCloud(cloud, cloudName);
    }
  }

  void Display::displayPointCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>::ConstPtr handler, const std::string& cloudName, const std::string& windowName) {
    //scoped lock
    boost::lock_guard<boost::mutex> lock(display_mutex);

    Display3D_Map::const_iterator itr_3d = displays3d.find(windowName);
    if (displays3d.end() != itr_3d) {
      (*itr_3d).second->setPointCloud(cloud, handler, cloudName);
    }
  }

  void Display::displayPolygonMesh(pcl::PolygonMesh::ConstPtr mesh, const std::string& meshName, const std::string& windowName) {
    //scoped lock
    boost::lock_guard<boost::mutex> lock(display_mutex);

    Display3D_Map::const_iterator itr_3d = displays3d.find(windowName);
    if (displays3d.end() != itr_3d) {
      (*itr_3d).second->setPolygonMesh(mesh, meshName);
    }
  }

  void Display::displaySphere(const pcl::PointXYZ center, const double& radius, const std::string& sphereName, const std::string& windowName) {
    //scoped lock
    boost::lock_guard<boost::mutex> lock(display_mutex);

    Display3D_Map::const_iterator itr_3d = displays3d.find(windowName);
    if (displays3d.end() != itr_3d) {
      (*itr_3d).second->setSphere(center, radius, sphereName);
    }
  }

  void Display::displayPlane(const pcl::ModelCoefficients& coeffs, const std::string& planeName, const std::string& windowName) {
    //scoped lock
    boost::lock_guard<boost::mutex> lock(display_mutex);

    Display3D_Map::const_iterator itr_3d = displays3d.find(windowName);
    if (displays3d.end() != itr_3d) {
      (*itr_3d).second->setPlane(coeffs, planeName);
    }
  }

  void Display::removePointCloud(const std::string& cloudName, const std::string& windowName) {
    //scoped lock
    boost::lock_guard<boost::mutex> lock(display_mutex);

    Display3D_Map::const_iterator itr_3d = displays3d.find(windowName);
    if (displays3d.end() != itr_3d) {
      (*itr_3d).second->removePointCloud(cloudName);
    }
  }

  void Display::removePolygonMesh(const std::string& meshName, const std::string& windowName) {
    //scoped lock
    boost::lock_guard<boost::mutex> lock(display_mutex);

    Display3D_Map::const_iterator itr_3d = displays3d.find(windowName);
    if (displays3d.end() != itr_3d) {
      (*itr_3d).second->removePolygonMesh(meshName);
    }
  }

  void Display::removeSphere(const std::string& sphereName, const std::string& windowName) {
    //scoped lock
    boost::lock_guard<boost::mutex> lock(display_mutex);

    Display3D_Map::const_iterator itr_3d = displays3d.find(windowName);
    if (displays3d.end() != itr_3d) {
      (*itr_3d).second->removeSphere(sphereName);
    }
  }

  void Display::removePlane(const std::string& planeName, const std::string& windowName) {
    //scoped lock
    boost::lock_guard<boost::mutex> lock(display_mutex);

    Display3D_Map::const_iterator itr_3d = displays3d.find(windowName);
    if (displays3d.end() != itr_3d) {
      (*itr_3d).second->removePlane(planeName);
    }
  }

  bool Display::windowExists(const std::string& name) {
    //scoped lock
    boost::lock_guard<boost::mutex> lock(display_mutex);

    Display2D_Map::const_iterator itr_2d = displays2d.find(name);
    if (displays2d.end() != itr_2d) {
      return true;
    }

    Display3D_Map::const_iterator itr_3d = displays3d.find(name);
    if (displays3d.end() != itr_3d) {
      return true;
    }

    return false;
  }

  void Display::createWindowIfDoesNotExist(const std::string& name) {
    //scoped lock
    boost::lock_guard<boost::mutex> lock(display_mutex);

    Display2D_Map::const_iterator itr_2d = displays2d.find(name);
    if (displays2d.end() == itr_2d) { //doesn't exist, create it
      DisplayWindow2D* newDisplayWindow2D = new DisplayWindow2D(name);
      displays2d[name] = newDisplayWindow2D;
    }

    Display3D_Map::const_iterator itr_3d = displays3d.find(name);
    if (displays3d.end() == itr_3d) { //doesn't exist, create it
      DisplayWindow3D* newDisplayWindow3D = new DisplayWindow3D(name);
      displays3d[name] = newDisplayWindow3D;
    }
  }

  void Display::destroyWindowIfItExists(const std::string& name) {
    //scoped lock
    boost::lock_guard<boost::mutex> lock(display_mutex);

    Display2D_Map::const_iterator itr_2d = displays2d.find(name);
    if (displays2d.end() != itr_2d) { //doesn't exist, create it
      delete (*itr_2d).second; //call delete on "new"ed DisplayWindow2D
      displays2d.erase(name); //remove from list
    }

    Display3D_Map::const_iterator itr_3d = displays3d.find(name);
    if (displays3d.end() != itr_3d) { //doesn't exist, create it
      delete (*itr_3d).second; //call delete on "new"ed DisplayWindow2D
      displays3d.erase(name); //remove from list
    }
  }

  void Display::cleanupDisplayWindows() {
    //scoped lock
    boost::lock_guard<boost::mutex> lock(display_mutex);

    Display2D_Map::const_iterator itr_2d;
    for (itr_2d = displays2d.begin(); itr_2d != displays2d.end(); ++itr_2d) {
      delete (*itr_2d).second; //call delete on "new"ed displayWindow2D
    }

    Display3D_Map::const_iterator itr_3d;
    for (itr_3d = displays3d.begin(); itr_3d != displays3d.end(); ++itr_3d) {
      delete (*itr_3d).second; //call delete on "new"ed DisplayWindow3D
    }
  }


  /////////// Display Window 2D //////////////////////////////////////////

  DisplayWindow2D::DisplayWindow2D(const std::string& name)
  : windowName(name),
  initialized(false),
  frameToDraw() {
  }

  DisplayWindow2D::~DisplayWindow2D() {
    if (initialized) {
      cv::destroyWindow(windowName);
    }
  }

  void DisplayWindow2D::setDisplayFrame(const cv::Mat& newFrameToDraw) {
    newFrameToDraw.copyTo(frameToDraw);
  }

  bool DisplayWindow2D::display() {
    try {
      if (!initialized && !frameToDraw.empty()) {
        cv::namedWindow(windowName, cv::WINDOW_AUTOSIZE);
        initialized = true;
      } else if (frameToDraw.empty()) {
        return false;
      }

      cv::imshow(windowName, frameToDraw);
      return true;

    } catch (...) {
      LOG4CXX_ERROR(Display::logger, "[DisplayWindow2D::display] Catch All.");
      return false;
    }
  }

  std::string DisplayWindow2D::getName() const {
    return windowName;
  };

  /////////// Display Window 3D //////////////////////////////////////////
  DisplayWindow3D::DisplayWindow3D(const std::string& name)
  : windowName(name),
  initialized(false),
  viewer(NULL),
  clouds_add(),
  clouds_update(),
  clouds_remove(),
  colorClouds_add(),
  colorClouds_update(),
  cloud_handlers(),
  meshes_add(),
  meshes_update(),
  meshes_remove(),
  spheres_add(),
  spheres_update(),
  spheres_remove(),
  planes_add(),
  planes_update(),
  planes_remove() {
  }

  DisplayWindow3D::~DisplayWindow3D() {
    if (initialized) {
      viewer->close(); // this is broken until vtk 8.0.1 is released
      delete viewer;
    }
  }

  std::string DisplayWindow3D::getName() const {
    return windowName;
  };

  void DisplayWindow3D::setPointCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloudToDisplay, const std::string& cloudName) {
    Cloud_Map::iterator itr_cloud = clouds_update.find(cloudName);
    if (clouds_update.end() == itr_cloud) { //if not in update list, add it to err... the add list
      clouds_add[cloudName] = cloudToDisplay->makeShared();
    } else { //update existing cloud
      (*itr_cloud).second = cloudToDisplay->makeShared();
    }
  }

  void DisplayWindow3D::setPointCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloudToDisplay, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>::ConstPtr handler, const std::string& cloudName) {
    setPointCloud(cloudToDisplay, cloudName);
    cloud_handlers[cloudName] = handler;
  }

  void DisplayWindow3D::setPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloudToDisplay, const std::string& cloudName) {
    ColorCloud_Map::iterator itr_cloud = colorClouds_update.find(cloudName);
    if (colorClouds_update.end() == itr_cloud) { //if not in update list, add it to err... the add list
      colorClouds_add[cloudName] = cloudToDisplay->makeShared();
    } else { //update existing cloud
      (*itr_cloud).second = cloudToDisplay->makeShared();
    }
  }

  void DisplayWindow3D::setPolygonMesh(pcl::PolygonMesh::ConstPtr meshToDisplay, const std::string& meshName) {
    Mesh_Map::iterator itr_mesh = meshes_update.find(meshName);
    if (meshes_update.end() == itr_mesh) { //if not in update list, add it to err... add list
      meshes_add[meshName] = meshToDisplay;
    } else { //update existing mesh
      (*itr_mesh).second = meshToDisplay;
    }
  }

  void DisplayWindow3D::setSphere(const pcl::PointXYZ center, const double& radius, const std::string& sphereName) {
    pcl::ModelCoefficients::Ptr sphereCoeffs(new pcl::ModelCoefficients);
    sphereCoeffs->values.resize(4);
    sphereCoeffs->values[0] = center.x;
    sphereCoeffs->values[1] = center.y;
    sphereCoeffs->values[2] = center.z;
    sphereCoeffs->values[3] = radius;
    Shape_Map::iterator itr_sphere = spheres_update.find(sphereName);
    if (spheres_update.end() == itr_sphere) { //if not in update list, add it to err... add list
      spheres_add[sphereName] = sphereCoeffs;
    } else { //update existing mesh
      (*itr_sphere).second = sphereCoeffs;
    }
  }

  void DisplayWindow3D::setPlane(const pcl::ModelCoefficients& coeffs, const std::string& planeName) {
    //EAK: this is kind of silly...
    pcl::ModelCoefficients::Ptr planeCoeffs(new pcl::ModelCoefficients);
    planeCoeffs->values.resize(4);
    planeCoeffs->values[0] = coeffs.values[0];
    planeCoeffs->values[1] = coeffs.values[1];
    planeCoeffs->values[2] = coeffs.values[2];
    planeCoeffs->values[3] = coeffs.values[3];
    Shape_Map::iterator itr_planes = planes_update.find(planeName);
    if (planes_update.end() == itr_planes) { //if not in update list, add it to err... add list
      planes_add[planeName] = planeCoeffs;
    } else { //update existing plane
      (*itr_planes).second = planeCoeffs;
    }
  }

  void DisplayWindow3D::removePointCloud(const std::string& cloudName) {
    //remove from clouds
    Cloud_Map::iterator itr_cloud = clouds_update.find(cloudName);
    if (clouds_update.end() != itr_cloud) {
      clouds_update.erase(itr_cloud);
    }
    itr_cloud = clouds_add.find(cloudName);
    if (clouds_add.end() != itr_cloud) {
      clouds_add.erase(itr_cloud);
    }

    //remove from color clouds
    ColorCloud_Map::iterator itr_color_cloud = colorClouds_update.find(cloudName);
    if (colorClouds_update.end() == itr_color_cloud) {
      colorClouds_update.erase(itr_color_cloud);
    }
    itr_color_cloud = colorClouds_add.find(cloudName);
    if (colorClouds_add.end() == itr_color_cloud) {
      colorClouds_add.erase(itr_color_cloud);
    }

    //remove from cloud handlers
    CloudHandler_Map::iterator itr_handler = cloud_handlers.find(cloudName);
    if (cloud_handlers.end() == itr_handler) {
      cloud_handlers.erase(itr_handler);
    }

    clouds_remove.push_back(cloudName);
  }

  void DisplayWindow3D::removePolygonMesh(const std::string& meshName) {
    //remove from meshes
    Mesh_Map::iterator itr_mesh = meshes_update.find(meshName);
    if (meshes_update.end() != itr_mesh) {
      meshes_update.erase(itr_mesh);
    }
    itr_mesh = meshes_add.find(meshName);
    if (meshes_add.end() != itr_mesh) {
      meshes_add.erase(itr_mesh);
    }

    meshes_remove.push_back(meshName);
  }

  void DisplayWindow3D::removeSphere(const std::string& sphereName) {
    //remove from spheres
    Shape_Map::iterator itr_sphere = spheres_update.find(sphereName);
    if (spheres_update.end() != itr_sphere) {
      spheres_update.erase(itr_sphere);
    }
    itr_sphere = spheres_add.find(sphereName);
    if (spheres_add.end() != itr_sphere) {
      spheres_add.erase(itr_sphere);
    }

    spheres_remove.push_back(sphereName);
  }

  void DisplayWindow3D::removePlane(const std::string& planeName) {
    //remove from planes
    Shape_Map::iterator itr = planes_update.find(planeName);
    if (planes_update.end() != itr) {
      planes_update.erase(itr);
    }
    itr = planes_add.find(planeName);
    if (planes_add.end() != itr) {
      planes_add.erase(itr);
    }

    planes_remove.push_back(planeName);
  }

  bool DisplayWindow3D::display() {
    try {
      if (!initialized && (!clouds_add.empty() || !colorClouds_add.empty() || !meshes_add.empty())) {
        viewer = new pcl::visualization::PCLVisualizer(windowName);
        viewer->initCameraParameters();
        viewer->addCoordinateSystem(0.1, "camera", 0);
        viewer->setCameraPosition(0, 0, -1, 0, 0, 0, 0, -1, 0);
        initialized = true;
      } else if (clouds_update.empty() && colorClouds_update.empty() && meshes_update.empty()) {
        return false;
      }

      //update existing point clouds in viewer
      Cloud_Map::const_iterator itr_cloud;
      for (itr_cloud = clouds_update.begin(); itr_cloud != clouds_update.end(); ++itr_cloud) {
        CloudHandler_Map::const_iterator itr_cloud_hander = cloud_handlers.find((*itr_cloud).first);
        if (cloud_handlers.end() != itr_cloud_hander) { //if there's a handler, use it
          viewer->updatePointCloud((*itr_cloud).second, *((*itr_cloud_hander).second), (*itr_cloud).first);
        } else {
          viewer->updatePointCloud((*itr_cloud).second, (*itr_cloud).first);
        }
      }

      //add new point clouds to viewer and transition clouds to update list
      for (itr_cloud = clouds_add.begin(); itr_cloud != clouds_add.end(); ++itr_cloud) {
        CloudHandler_Map::const_iterator itr_cloud_hander = cloud_handlers.find((*itr_cloud).first);
        if (cloud_handlers.end() != itr_cloud_hander) { //if there's a handler, use it
          viewer->addPointCloud((*itr_cloud).second, *((*itr_cloud_hander).second), (*itr_cloud).first);
        } else {
          viewer->addPointCloud((*itr_cloud).second, (*itr_cloud).first);
        }
        clouds_update[(*itr_cloud).first] = (*itr_cloud).second;
      }
      clouds_add.clear();


      //update existing color point clouds in viewer
      ColorCloud_Map::const_iterator itr_colorCloud;
      for (itr_colorCloud = colorClouds_update.begin(); itr_colorCloud != colorClouds_update.end(); ++itr_colorCloud) {
        viewer->updatePointCloud((*itr_colorCloud).second, (*itr_colorCloud).first);
      }

      //add new color point clouds to viewer and transition clouds to update list
      for (itr_colorCloud = colorClouds_add.begin(); itr_colorCloud != colorClouds_add.end(); ++itr_colorCloud) {
        viewer->addPointCloud((*itr_colorCloud).second, (*itr_colorCloud).first);
        colorClouds_update[(*itr_colorCloud).first] = (*itr_colorCloud).second;
      }
      colorClouds_add.clear();

      //remove clouds
      std::vector<std::string>::iterator itr;
      for (itr = clouds_remove.begin(); itr != clouds_remove.end(); ++itr) {
        viewer->removePointCloud(*itr);
      }
      clouds_remove.clear();
      ///////////////////////////////////////////////////////////////

      //// update existing polygon meshes in viewer /////////////////
      Mesh_Map::const_iterator itr_mesh;
      for (itr_mesh = meshes_update.begin(); itr_mesh != meshes_update.end(); ++itr_mesh) {
        //viewer->updatePolygonMesh((*itr_mesh).second);
        viewer->removeShape((*itr_mesh).first);
        viewer->addPolygonMesh((*(*itr_mesh).second), (*itr_mesh).first);
      }

      //add polygon meshes to viewer and transition meshes to update list
      for (itr_mesh = meshes_add.begin(); itr_mesh != meshes_add.end(); ++itr_mesh) {
        viewer->addPolygonMesh((*(*itr_mesh).second), (*itr_mesh).first);
        meshes_update[(*itr_mesh).first] = (*itr_mesh).second;
      }
      meshes_add.clear();

      //remove meshes
      for (itr = meshes_remove.begin(); itr != meshes_remove.end(); ++itr) {
        viewer->removePolygonMesh(*itr);
      }
      meshes_remove.clear();
      ///////////////////////////////////////////////////////////////

      //// update existing spheres in viewer ////////////////////////
      Shape_Map::const_iterator itr_sphere;
      for (itr_sphere = spheres_update.begin(); itr_sphere != spheres_update.end(); ++itr_sphere) {
        //viewer->updateSphere((*itr_sphere).second);
        viewer->removeShape((*itr_sphere).first);
        viewer->addSphere((*(*itr_sphere).second), (*itr_sphere).first);
      }

      //add spheres to viewer and transition spheres to update list
      for (itr_sphere = spheres_add.begin(); itr_sphere != spheres_add.end(); ++itr_sphere) {
        viewer->addSphere((*(*itr_sphere).second), (*itr_sphere).first);
        spheres_update[(*itr_sphere).first] = (*itr_sphere).second;
      }
      spheres_add.clear();

      //remove spheres
      for (itr = spheres_remove.begin(); itr != spheres_remove.end(); ++itr) {
        viewer->removeShape(*itr);
      }
      spheres_remove.clear();
      ///////////////////////////////////////////////////////////////

      //// update existing planes in viewer //////////////////////////
      Shape_Map::const_iterator itr_plane;
      for (itr_plane = planes_update.begin(); itr_plane != planes_update.end(); ++itr_plane) {
        //viewer->updateSphere((*itr_plane).second);
        viewer->removeShape((*itr_plane).first);
        viewer->addPlane((*(*itr_plane).second), (*itr_plane).first);
      }

      //add planes to viewer and transition planes to update list
      for (itr_plane = planes_add.begin(); itr_plane != planes_add.end(); ++itr_plane) {
        viewer->addPlane((*(*itr_plane).second), (*itr_plane).first);
        planes_update[(*itr_plane).first] = (*itr_plane).second;
      }
      planes_add.clear();

      //remove planes
      for (itr = planes_remove.begin(); itr != planes_remove.end(); ++itr) {
        viewer->removeShape(*itr);
      }
      planes_remove.clear();
      ///////////////////////////////////////////////////////////////

      //display
      //printf("[DisplayWindow3D::display] spin once.\n");
      viewer->spinOnce();
      return true;

    } catch (...) {
      cout << "[DisplayWindow3D::display] Catch All." << endl;
      return false;
    }
  }

} //namespace ade
