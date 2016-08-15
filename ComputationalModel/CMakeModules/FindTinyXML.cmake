# - Find TinyXML
# Find the native TinyXML includes and library
#
#   TINYXML_FOUND       - True if TinyXML found.
#   TINYXML_INCLUDE_DIR - where to find tinyxml.h, etc.
#   TINYXML_LIBRARIES   - List of libraries when using TinyXML.

if (WIN32)
    find_path( TINYXML_INCLUDE_DIR
            NAMES
                "tinyxml.h"
            PATHS
                ${PROJECT_SOURCE_DIR}/tinyxml/include
				${PROJECT_SOURCE_DIR}/visualStudio_2015/tinyxml/include
            )
    if(ARCH STREQUAL "x86")
        find_library( TinyXML_LIBRARIES
                NAMES
                    tinyxml
                PATHS
                    ${PROJECT_SOURCE_DIR}/tinyxml/lib/win32
                    ${PROJECT_SOURCE_DIR}/tinyxml/visualStudio_2015/lib/win32
                )
    else()
        find_library( TinyXML_LIBRARIES
                NAMES
                    tinyxml
                PATHS
                    ${PROJECT_SOURCE_DIR}/tinyxml/lib/win32
                    ${PROJECT_SOURCE_DIR}/visualStudio_2015/tinyxml/lib/win32
                )
    endif()

else()
#    FIND_PATH( TinyXML_INCLUDE_DIR "tinyxml.h" PATH_SUFFIXES "tinyxml" )
#    FIND_LIBRARY( TinyXML_LIBRARIES NAMES "tinyxml" PATH_SUFFIXES "tinyxml" )
endif()


# handle the QUIETLY and REQUIRED arguments and set TINYXML_FOUND to TRUE if
# all listed variables are TRUE
INCLUDE( "FindPackageHandleStandardArgs" )
FIND_PACKAGE_HANDLE_STANDARD_ARGS( "TinyXML" DEFAULT_MSG TINYXML_INCLUDE_DIR TinyXML_LIBRARIES )

MARK_AS_ADVANCED( TINYXML_INCLUDE_DIR TinyXML_LIBRARIES )
