cmake_minimum_required(VERSION 3.5.0)
project(mimpc)

option(BuildSim "BuildSim" ON)
option(BuildExamples "BuildExamples" ON)

if(NOT CMAKE_BUILD_TYPE)
   set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-Ofast")

set(LIB_VERSION_MAJOR 0 CACHE STRING "major version" FORCE)
set(LIB_VERSION_MINOR 1 CACHE STRING "minor version" FORCE)
set(LIB_VERSION ${LIB_VERSION_MAJOR}.${LIB_VERSION_MINOR} CACHE STRING "version" FORCE)

message("Building mimpc library")
set(CMAKE_CXX_STANDARD 23)
set(LIBSOURCES
        src/systems/REACSA.cpp
)

find_package(fmt REQUIRED)
find_package(SCIP REQUIRED)

if(BuildSim)
   message("Building simulation related content")
   set(SIMSOURCES
           src/sim/VizForces.cpp
   )
   find_package(drake CONFIG REQUIRED)
   find_package(ZLIB)
   add_library(cnpy SHARED IMPORTED) # or STATIC instead of SHARED
   set_target_properties(cnpy PROPERTIES
   IMPORTED_LOCATION "/usr/local/lib/libcnpy.so"
   INTERFACE_INCLUDE_DIRECTORIES "/usr/local/include"
   )
endif()

set(LIB_INCLUDE_INSTALL_DESTINATION include/mimpc)
add_library(mimpclib ${LIBSOURCES})
target_include_directories(mimpclib PUBLIC
                                $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                                $<INSTALL_INTERFACE:${MYLB_INCLUDE_INSTALL_DESTINATION}>)
target_include_directories(mimpclib PUBLIC ${SCIP_INCLUDE_DIRS})
target_link_libraries(mimpclib PUBLIC ${SCIP_LIBRARIES})
target_link_libraries(mimpclib INTERFACE ${SCIP_LIBRARIES} fmt::fmt)

if (BuildSim)
   add_library(mimpclibsim ${SIMSOURCES})
   target_include_directories(mimpclibsim PUBLIC
           $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:${MYLB_INCLUDE_INSTALL_DESTINATION}>)
   target_link_libraries(mimpclibsim INTERFACE mimpclib)
   target_link_libraries(mimpclibsim PUBLIC drake::drake)
   target_link_libraries(mimpclibsim INTERFACE cnpy)
   target_link_libraries(mimpclibsim INTERFACE ZLIB::ZLIB)
   target_link_libraries(mimpclibsim INTERFACE fmt::fmt)
endif ()

if(BuildExamples)
   message("Building example related content")
   set(EXAMPLESOURCES
           examples/esa_reacsa/main.cpp
   )
   add_executable(reacsamimpc ${EXAMPLESOURCES})
   target_link_libraries(reacsamimpc mimpclibsim)
   target_include_directories(reacsamimpc PUBLIC ${SCIP_INCLUDE_DIRS})
   target_include_directories(reacsamimpc PUBLIC include)
endif()

INSTALL(TARGETS mimpclib EXPORT mimpclib
        # for executables and dll on Win
        RUNTIME DESTINATION bin
        # shared libraries
        LIBRARY DESTINATION lib
        # for static libraries
        ARCHIVE DESTINATION lib
)

install(DIRECTORY "${CMAKE_SOURCE_DIR}/include" # source directory
        DESTINATION "${LIB_INCLUDE_INSTALL_DESTINATION}" # target directory
)
        
INSTALL(EXPORT mimpclib
        DESTINATION lib/cmake/mimpclib
        FILE mimpclibConfig.cmake
        NAMESPACE mimpclib::
)

target_compile_options(mimpclib PRIVATE -Werror -Wall -Wextra -pedantic)

if(BuildSim)
   target_compile_options(mimpclibsim PRIVATE -Werror -Wall -Wextra -pedantic)
endif ()
