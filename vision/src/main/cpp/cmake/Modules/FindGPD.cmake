###########################################
#Find GPD library
#
#This helps find:
#> The header files for GPD library
#> The shared library libgpd.so
#
SET(HEADERS_GRASP
        clustering.h
        grasp_detector.h
        sequential_importance_sampling.h
        data_generator.h
        candidate
        hand.h
        hand_set.h
        hand_geometry.h
        antipodal.h
        frame_estimator.h
        hand_search.h
        local_frame.h
        finger_hand.h
        candidates_generator.h
)

# TODO: should these headers be included?
#descriptor/image_3_channels_strategy.h
#descriptor/image_generator.h
#descriptor/image_12_channels_strategy.h
#descriptor/image_1_channels_strategy.h
#descriptor/image_geometry.h
#descriptor/image_strategy.h
#descriptor/image_15_channels_strategy.h
#net/caffe_classifier.h
#net/conv_layer.h
#net/dense_layer.h
#net/eigen_classifier.h
#net/layer.h
#net/classifier.h
#net/openvino_classifier.h.in
#util/cloud.h
#util/plot.h
#util/point_list.h
#util/config_file.h
#util/eigen_utils.h

FIND_PATH(GPD_INCLUDE_DIR ${HEADERS_GRASP}
        PATH /usr/local/include/gpd)

FIND_LIBRARY(GPD_LIBRARY libgpd.so
        PATH /usr/local/lib)

SET(GPD_LIBRARIES ${GPD_LIBRARY})
SET(GPD_INCLUDE_DIRS ${GPD_INCLUDE_DIR})

message("GPD_INCLUDE_DIR: ${GPD_INCLUDE_DIR}")

INCLUDE(FindPackageHandleStandardArgs)
# check that all listed variables are TRUE
find_package_handle_standard_args(GPD DEFAULT_MSG
        GPD_LIBRARY GPD_INCLUDE_DIR)

mark_as_advanced(GPD_INCLUDE_DIR GPD_LIBRARY)
