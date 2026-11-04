PROJECT(grasp)

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})

SET(SOURCES
    GraspInterface.cpp
    GraspDetector.cpp
	SmallObjectGrasp.cpp
	)

# for downloading models
SET(MODEL_DIR "${CMAKE_SOURCE_DIR}/../../../data/grasp/")

if (AGILEGRASP_FOUND)
	SET(SOURCES ${SOURCES}
		AgileGrasp.cpp
	)
endif(AGILEGRASP_FOUND)

if (GPD_FOUND)
    SET(SOURCES ${SOURCES}
        GpdGrasp.cpp
    )
    message(STATUS "Source directory: ${CMAKE_SOURCE_DIR}")

    # download models and config files from GPD repo
    SET(GPD_MODEL_BASE_URL "https://github.com/atenpas/gpd/raw/master/models/lenet/15channels/params")
    SET(GPD_MODEL_PATH "${MODEL_DIR}gpd/models/lenet/15channels/params")

    # Define the list of files to download (URL; Destination File)
    set(FILES_TO_DOWNLOAD
            "${GPD_MODEL_BASE_URL}/conv1_biases.bin ${GPD_MODEL_PATH}/conv1_biases.bin"
            "${GPD_MODEL_BASE_URL}/conv1_weights.bin ${GPD_MODEL_PATH}/conv1_weights.bin"
            "${GPD_MODEL_BASE_URL}/conv2_biases.bin ${GPD_MODEL_PATH}/conv2_biases.bin"
            "${GPD_MODEL_BASE_URL}/conv2_weights.bin ${GPD_MODEL_PATH}/conv2_weights.bin"
            "${GPD_MODEL_BASE_URL}/ip1_biases.bin ${GPD_MODEL_PATH}/ip1_biases.bin"
            "${GPD_MODEL_BASE_URL}/ip1_weights.bin ${GPD_MODEL_PATH}/ip1_weights.bin"
            "${GPD_MODEL_BASE_URL}/ip2_biases.bin ${GPD_MODEL_PATH}/ip2_biases.bin"
            "${GPD_MODEL_BASE_URL}/ip2_weights.bin ${GPD_MODEL_PATH}/ip2_weights.bin"
    )

    # Ensure the download directory exists
    file(MAKE_DIRECTORY "${GPD_MODEL_PATH}")

    # Loop through the list and download each file
    foreach(FILE_PAIR IN LISTS FILES_TO_DOWNLOAD)
        # Split the combined string into URL and Destination

        separate_arguments(FILE_PAIR)
        list(GET FILE_PAIR 0 URL)
        list(GET FILE_PAIR 1 DESTINATION)

        # Download the file
        if (NOT EXISTS "${DESTINATION}")
            message(STATUS "Downloading: ${URL} to ${DESTINATION}")
            file(DOWNLOAD
                    ${URL}
                    ${DESTINATION}
                    SHOW_PROGRESS # Optional: display download progress
            )
        endif(NOT EXISTS "${DESTINATION}")
    endforeach()
endif(GPD_FOUND)

ADD_LIBRARY(${PROJECT_NAME} SHARED ${SOURCES})

# link libraries
TARGET_LINK_LIBRARIES(${PROJECT_NAME} common stm)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${LOG4CXX_LIBRARIES} ${JSONCPP_LIBRARY})
if (AGILEGRASP_FOUND)
    TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${AGILEGRASP_LIBRARIES})
endif (AGILEGRASP_FOUND)
if (GPD_FOUND)
    TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${GPD_LIBRARIES})
endif (GPD_FOUND)

# build java wrappers via swig
ADD_SUBDIRECTORY(swig)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} grasp_swig)
