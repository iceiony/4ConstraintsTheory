# App_BasicExample is a minimal sample creating, stepping and deleting a Bullet dynamics world

INCLUDE_DIRECTORIES(
        ${BULLET_PHYSICS_SOURCE_DIR}/src
        ${BULLET_PHYSICS_SOURCE_DIR}/examples/ThirdPartyLibs
)


SET(BulletDemo_SRC
        ../Importers/ImportObjDemo/LoadMeshFromObj.h
        ../Importers/ImportObjDemo/Wavefront2GLInstanceGraphicsShape.h
        ../Importers/ImportObjDemo/LoadMeshFromObj.cpp
        ../Importers/ImportObjDemo/Wavefront2GLInstanceGraphicsShape.cpp

        ../ThirdPartyLibs/Wavefront/tiny_obj_loader.cpp
        ../ThirdPartyLibs/tinyxml/tinystr.cpp
        ../ThirdPartyLibs/tinyxml/tinyxml.cpp
        ../ThirdPartyLibs/tinyxml/tinyxmlerror.cpp
        ../ThirdPartyLibs/tinyxml/tinyxmlparser.cpp

        ../ThirdPartyLibs/tinyxml/tinystr.cpp
        ../ThirdPartyLibs/tinyxml/tinyxml.cpp
        ../ThirdPartyLibs/tinyxml/tinyxmlerror.cpp
        ../ThirdPartyLibs/tinyxml/tinyxmlparser.cpp

        ../Utils/b3Clock.cpp
        ../Utils/b3Clock.h
        ../Utils/b3ResourcePath.cpp
        ../Utils/b3ResourcePath.h
        ${BULLET_PHYSICS_SOURCE_DIR}/build3/bullet.rc
        )

ADD_LIBRARY(BulletDemoLib ${BulletDemo_SRC} )
SET_TARGET_PROPERTIES(BulletDemoLib PROPERTIES VERSION ${BULLET_VERSION})
SET_TARGET_PROPERTIES(BulletDemoLib PROPERTIES SOVERSION ${BULLET_VERSION})

file(COPY "lego_tool1.obj" DESTINATION "./")

LINK_LIBRARIES(
 BulletDemoLib BulletDynamics BulletCollision LinearMath Bullet3Common
)

IF (WIN32)
	ADD_EXECUTABLE(AppBasicExample
		BasicExample.cpp
		main.cpp
		${BULLET_PHYSICS_SOURCE_DIR}/build3/bullet.rc
	)
ELSE()
	ADD_EXECUTABLE(AppBasicExample
		BasicExample.cpp
		main.cpp
	)
ENDIF()




IF (INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)
			SET_TARGET_PROPERTIES(App_BasicExample PROPERTIES  DEBUG_POSTFIX "_Debug")
			SET_TARGET_PROPERTIES(App_BasicExample PROPERTIES  MINSIZEREL_POSTFIX "_MinsizeRel")
			SET_TARGET_PROPERTIES(App_BasicExample PROPERTIES  RELWITHDEBINFO_POSTFIX "_RelWithDebugInfo")
ENDIF(INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)





#################
# Standalone BasicExampleGui using OpenGL (but not the example browser)


INCLUDE_DIRECTORIES(
        ${BULLET_PHYSICS_SOURCE_DIR}/src
        ${BULLET_PHYSICS_SOURCE_DIR}/btgui
        ${BULLET_PHYSICS_SOURCE_DIR}/examples
        ${BULLET_PHYSICS_SOURCE_DIR}/examples/ThirdPartyLibs/Glew
        ${BULLET_PHYSICS_SOURCE_DIR}/examples/ThirdPartyLibs
)

SET(AppBasicExampleGui_SRCS
	BasicExample.cpp
    ${BULLET_PHYSICS_SOURCE_DIR}/build3/bullet.rc
    ../StandaloneMain/main_opengl_single_example.cpp
    ../ExampleBrowser/OpenGLGuiHelper.cpp
    ../ExampleBrowser/GL_ShapeDrawer.cpp
)

#this define maps StandaloneExampleCreateFunc to the right 'CreateFunc'
ADD_DEFINITIONS(-DB3_USE_STANDALONE_EXAMPLE)

LINK_LIBRARIES(
        BulletDemoLib BulletDynamics BulletCollision LinearMath OpenGLWindow Bullet3Common ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY}
)

#some code to support OpenGL and Glew cross platform
IF (WIN32)
        INCLUDE_DIRECTORIES(
             ${BULLET_PHYSICS_SOURCE_DIR}/btgui/OpenGLWindow/GlewWindows
        )
        ADD_DEFINITIONS(-DGLEW_STATIC)
ELSE(WIN32)
        IF(APPLE)
                find_library(COCOA NAMES Cocoa)
                MESSAGE(${COCOA})
                link_libraries(${COCOA})

        ELSE(APPLE)
                INCLUDE_DIRECTORIES(
                        ${BULLET_PHYSICS_SOURCE_DIR}/btgui/OpenGLWindow/GlewWindows
                )
                ADD_DEFINITIONS("-DGLEW_INIT_OPENGL11_FUNCTIONS=1")
                ADD_DEFINITIONS("-DGLEW_STATIC")
                ADD_DEFINITIONS("-DGLEW_DYNAMIC_LOAD_ALL_GLX_FUNCTIONS=1")

                LINK_LIBRARIES( X11 pthread dl Xext)
        ENDIF(APPLE)
ENDIF(WIN32)


ADD_EXECUTABLE(AppBasicExampleGui
                ${AppBasicExampleGui_SRCS}
)


IF (INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)
                        SET_TARGET_PROPERTIES(AppBasicExampleGui PROPERTIES  DEBUG_POSTFIX "_Debug")
                        SET_TARGET_PROPERTIES(AppBasicExampleGui PROPERTIES  MINSIZEREL_POSTFIX "_MinsizeRel")
                        SET_TARGET_PROPERTIES(AppBasicExampleGui PROPERTIES  RELWITHDEBINFO_POSTFIX "_RelWithDebugInfo")
ENDIF(INTERNAL_ADD_POSTFIX_EXECUTABLE_NAMES)


