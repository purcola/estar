### Simplified CMake build file for E-Star.
###
### We only build the core library with CMake, no PNF or fancy test
### programs. If you want any of those, use the GNU Autoconf /
### Automake build system.

PROJECT (E-Star)
SET (CMAKE_VERBOSE_MAKEFILE ON)

MESSAGE (STATUS "System: ${CMAKE_SYSTEM_NAME}")



### Some compiler flags are not understood by all...

INCLUDE (CheckCXXCompilerFlag)

CHECK_CXX_COMPILER_FLAG (-O0 ESTAR_CXX_FLAG_O)
IF (ESTAR_CXX_FLAG_O)
  ADD_DEFINITIONS (-O0)
ENDIF (ESTAR_CXX_FLAG_O)

CHECK_CXX_COMPILER_FLAG (-g ESTAR_CXX_FLAG_g)
IF (ESTAR_CXX_FLAG_g)
  ADD_DEFINITIONS (-g)
ENDIF (ESTAR_CXX_FLAG_g)

CHECK_CXX_COMPILER_FLAG (-Wall ESTAR_CXX_FLAG_Wall)
IF (ESTAR_CXX_FLAG_Wall)
  ADD_DEFINITIONS (-Wall)
ENDIF (ESTAR_CXX_FLAG_Wall)

CHECK_CXX_COMPILER_FLAG (-Wno-long-double ESTAR_CXX_FLAG_Wnld)
IF (ESTAR_CXX_FLAG_Wnld)
  ADD_DEFINITIONS (-Wno-long-double)
ENDIF (ESTAR_CXX_FLAG_Wnld)

### -Wundef can sometimes help you find errors when porting files from
### Windows, but it can be *very* noisy in Boost headers
# CHECK_CXX_COMPILER_FLAG (-Wundef ESTAR_CXX_FLAG_Wundef)
# IF (ESTAR_CXX_FLAG_Wundef)
#   ADD_DEFINITIONS (-Wundef)
# ENDIF (ESTAR_CXX_FLAG_Wundef)



### Find dependencies

INCLUDE (FindBoost)

IF (Boost_FOUND)
  MESSAGE (STATUS "Found Boost in ${Boost_INCLUDE_DIRS}")
  INCLUDE_DIRECTORIES (${Boost_INCLUDE_DIRS})
ELSE (Boost_FOUND)
  MESSAGE (FATAL_ERROR "Boost libraries not found")
ENDIF (Boost_FOUND)

INCLUDE (CheckIncludeFileCXX)

SET (CMAKE_REQUIRED_INCLUDES ${Boost_INCLUDE_DIRS})

CHECK_INCLUDE_FILE_CXX (boost/shared_ptr.hpp ESTAR_HAVE_SHARED_PTR)
IF (${ESTAR_HAVE_SHARED_PTR})
ELSE (${ESTAR_HAVE_SHARED_PTR})
      MESSAGE(FATAL_ERROR "boost/shared_ptr.hpp not found")
ENDIF (${ESTAR_HAVE_SHARED_PTR})

CHECK_INCLUDE_FILE_CXX (boost/graph/adjacency_list.hpp ESTAR_HAVE_ADJLIST)
IF (${ESTAR_HAVE_ADJLIST})
ELSE (${ESTAR_HAVE_ADJLIST})
      MESSAGE(FATAL_ERROR "boost/graph/adjacency_list.hpp not found")
ENDIF (${ESTAR_HAVE_ADJLIST})

INCLUDE (FindOpenGL)

IF (OPENGL_FOUND)
  MESSAGE (STATUS "Found OpenGL in ${OPENGL_INCLUDE_DIR}")
  MESSAGE (STATUS "  OPENGL_LIBRARIES   ${OPENGL_LIBRARIES}")
  MESSAGE (STATUS "  OPENGL_gl_LIBRARY  ${OPENGL_gl_LIBRARY}")
  MESSAGE (STATUS "  OPENGL_glu_LIBRARY ${OPENGL_glu_LIBRARY}")
  INCLUDE_DIRECTORIES (${OPENGL_INCLUDE_DIR})
ELSE (OPENGL_FOUND)
  MESSAGE (FATAL_ERROR "OpenGL not found")
ENDIF (OPENGL_FOUND)

# INCLUDE (FindGLUT)
# 
# IF (GLUT_FOUND)
#   MESSAGE (STATUS "Found GLUT in ${GLUT_INCLUDE_DIR}")
#   MESSAGE (STATUS "  GLUT_LIBRARIES    ${GLUT_LIBRARIES}")
#   MESSAGE (STATUS "  GLUT_glut_LIBRARY ${GLUT_glut_LIBRARY}")
#   MESSAGE (STATUS "  GLUT_Xmu_LIBRARY  ${GLUT_Xmu_LIBRARY}")
#   MESSAGE (STATUS "  GLUT_Xi_LIBRARY   ${GLUT_Xi_LIBRARY}")
#   INCLUDE_DIRECTORIES (${GLUT_INCLUDE_DIR})
# ELSE (GLUT_FOUND)
#   MESSAGE (FATAL_ERROR "GLUT not found")
# ENDIF (GLUT_FOUND)



### Some OS-dependent flags.

IF (WIN32)
  MESSAGE (STATUS "Detected Microsoft Windows")
  ADD_DEFINITIONS (-DWIN32)
ELSE (WIN32)
  IF (APPLE)
    MESSAGE (STATUS "Detected Mac OS X")
    ADD_DEFINITIONS (-DOSX)
  ENDIF (APPLE)
  
  IF (CMAKE_SYSTEM_NAME MATCHES Linux)
    MESSAGE (STATUS "Detected Linux")
    ADD_DEFINITIONS (-DLINUX)
  ENDIF (CMAKE_SYSTEM_NAME MATCHES Linux)
  
  IF (CMAKE_SYSTEM_NAME MATCHES OpenBSD)
    MESSAGE (STATUS "Detected OpenBSD")
    ADD_DEFINITIONS (-DOPENBSD)
  ENDIF (CMAKE_SYSTEM_NAME MATCHES OpenBSD)
ENDIF(WIN32)



SET (PREFIX /usr/local CACHE PATH "project installation prefix")

SUBDIRS (estar gfx)
