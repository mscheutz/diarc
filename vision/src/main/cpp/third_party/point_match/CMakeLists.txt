cmake_minimum_required (VERSION 2.9)
PROJECT(shapecontext)

include_directories(.)

SET(SOURCE_CPP
Affine.cpp
Bookstein.cpp
Hungarian.cpp
Outlier.cpp
PointMatch.cpp
Relax.cpp
ShapeContext.cpp
Tools.cpp)

SET(SOURCE_H
PointMatch.h
Relax.h
Resource.h
StdAfx.h
Tools.h)

add_library(${PROJECT_NAME} SHARED ${SOURCE_H} ${SOURCE_CPP})

#add_executable(PointMatchDemo1 PointMatchDemo1.cpp)
#target_link_libraries (${BINARY_NAME} shapecontext)

#add_executable(PointMatchDemo1 PointMatchDemo1.cpp ${SOURCE_CPP})

#add_executable(PointMatchDemo2 PointMatchDemo2.cpp ${SOURCE_CPP})

