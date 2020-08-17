# CMake file for sba's demo program
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src )
LINK_LIBRARIES(${PROJECT_NAME} ${REQUIRED_LIBRARIES})
#ADD_EXECUTABLE(test_dummy test_dummy.cpp  )

#IF(NOT WIN32)


#ADD_EXECUTABLE(test_global_opt test_global_opt.cpp  )
##ADD_EXECUTABLE(test_show_mappoint_projections test_show_mappoint_projections.cpp  )
## ADD_EXECUTABLE(test_kfdatabase  test_kfdatabase.cpp  )

#ADD_EXECUTABLE(test_bow_reloc test_bow_reloc.cpp  )

##ADD_EXECUTABLE(test_graph test_graph.cpp  )
#ADD_EXECUTABLE(test_orbmur test_orbmur.cpp  )
##ADD_EXECUTABLE(test_fbow test_fbow.cpp  )

#ADD_EXECUTABLE(test_umat_feat test_umat_feat.cpp  )
#ADD_EXECUTABLE(test_arucoinit test_arucoinit.cpp  )
#ADD_EXECUTABLE(test_videojoin test_videojoin.cpp  )
##ADD_EXECUTABLE(test_arucotrackingfrommap test_arucotrackingfrommap.cpp  )
#ADD_EXECUTABLE(compare_logs compare_logs.cpp  )
#ADD_EXECUTABLE(compare_logs_kitti compare_logs_kitti.cpp kittieval/matrix.cpp  kittieval/matrix.h   )
#find_package(marker_mapper)
##IF (${marker_mapper_FOUND})
##ADD_EXECUTABLE(test_convertToMarkerMapper test_convertToMarkerMapper.cpp  )
##target_link_libraries(test_convertToMarkerMapper marker_mapper)
##ENDIF()
#ADD_EXECUTABLE(leica_error_frommap leica_error_frommap.cpp  )

#ADD_EXECUTABLE(slamuco2pcd slamuco2pcd.cpp  )
#ADD_EXECUTABLE(test_sgl test_sgl.cpp  )
##ADD_EXECUTABLE(test_calibrationObject test_calibrationObject.cpp  )
#ADD_EXECUTABLE(test_picoflannspeed test_picoflannspeed.cpp  )
#ADD_EXECUTABLE(test_essentialmatrix test_essentialmatrix.cpp  )

##ADD_EXECUTABLE(test_g2o test_g2o.cpp  )
##target_link_libraries(test_g2o    ${G2O_LIBS})
#ADD_EXECUTABLE(test_ferns test_ferns.cpp  )

##ADD_EXECUTABLE(test_markersOnPlane test_markersOnPlane.cpp  )

#ADD_EXECUTABLE(test_extensiblecontainer test_extensiblecontainer.cpp  )
#ADD_EXECUTABLE(test_nanogo test_nanogo.cpp  picoflann.h nanoflann.hpp nanogo.h)
#ADD_EXECUTABLE(test_picoflann test_picoflann.cpp  picoflann.h nanoflann.hpp)
#ADD_EXECUTABLE(avi2images avi2images.cpp  )
#ADD_EXECUTABLE(dso_cvt_cameraparams dso_cvt_cameraparams.cpp  )



#ENDIF()
#ADD_EXECUTABLE(kittieval kittieval/evaluate_odometry.cpp  kittieval/mail.h  kittieval/matrix.cpp  kittieval/matrix.h  )


#Find OpenNI2
### OPENNI 2
set(OPENNI2LIB_FOUND "NO" )

if(NOT WIN32)
  find_path(OpenNI2_INCLUDE  NAMES OpenNI2/OpenNI.h openni2/OpenNI.h)
  find_library(LIBOPENNI2_LIBRARY NAMES OpenNI2  )
else()
  find_path(OpenNI2_INCLUDE  NAMES OpenNI.h PATHS  ${OPENNI2_DIR}/Include/ )
  find_library(LIBOPENNI2_LIBRARY NAMES OpenNI2 PATHS ${OPENNI2_DIR}/Lib )
endif()

#message(FATAL_ERROR "NI=${OpenNI2_INCLUDE}")
if ( (OpenNI2_INCLUDE STREQUAL "OpenNI2_INCLUDE-NOTFOUND") OR (LIBOPENNI2_LIBRARY STREQUAL "LIBOPENNI2_LIBRARY-NOTFOUND"))
       message(STATUS_MESSAGE "OpenNi  not found inc=${OpenNI2_INCLUDE}")
  else()
    if (WIN32)
        include_directories(${OpenNI2_INCLUDE})
    else()
      include_directories(${OpenNI2_INCLUDE}/openni2)
    endif()
    message(STATUS  "OpenNI.h=${OpenNI2_INCLUDE} LIBOPENNI2_LIBRARY=${LIBOPENNI2_LIBRARY}")
    set(OPENNI2LIB_FOUND "YES" )
 endif()
 IF(${OPENNI2LIB_FOUND})
 ADD_EXECUTABLE(oni2avi oni2avi.cpp cvni2.h )
 TARGET_LINK_LIBRARIES(oni2avi ${LIBOPENNI2_LIBRARY}  )
 #ADD_EXECUTABLE(onirecord onirecord.cpp  )
 #TARGET_LINK_LIBRARIES(onirecord ${LIBOPENNI2_LIBRARY}  )
 ENDIF()



 ADD_EXECUTABLE(test_tum test_tum.cpp )
