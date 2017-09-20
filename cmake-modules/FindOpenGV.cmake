FIND_PATH(OPENGV_INCLUDE_DIR opengv/types.hpp
    ${OPENGV_ROOT}/include
    $ENV{OPENGV_ROOT}/include
    /usr/include
    /opt/local/include
    /usr/local/include
    /sw/include
  )

FIND_LIBRARY(OPENGV_LIBRARY opengv
     ${OPENGV_ROOT}/lib
     $ENV{OPENGV_ROOT}/lib
     /usr/lib
     /usr/local/lib
     /opt/local/lib
     /sw/lib
   )

IF(OPENGV_INCLUDE_DIR AND OPENGV_LIBRARY)
  SET(OPENGV_FOUND TRUE)
ELSE(OPENGV_INCLUDE_DIR AND OPENGV_LIBRARY)
  SET(OPENGV_FOUND FALSE)
ENDIF(OPENGV_INCLUDE_DIR AND OPENGV_LIBRARY)
