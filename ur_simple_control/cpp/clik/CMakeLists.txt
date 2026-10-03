# TODO idk what the actual minimum version is
cmake_minimum_required(VERSION 3.20.0)
project(clik)
set(PROJECT_DESCRIPTION
		"basic click(s) with ur_rtde and pinocchio"
)
set ( CMAKE_EXPORT_COMPILE_COMMANDS ON )

# TODO put this in
#set(PROJECT_URL https://github.com/${PROJECT_NAMESPACE}/${PROJECT_NAME})
# std=c++17 so that i can use std::filesystem, but otherwise min is 11
set(CMAKE_CXX_STANDARD 17)


# Build a static binary.
#set(CMAKE_EXE_LINKER_FLAGS "-static-libgcc -static-libstdc++")
# Add further warning levels, they only help
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} \
    -D_XOPEN_SOURCE=700 \
    -D_FORTIFY_SOURCE=2 \
    -O2 \
    -fstack-protector \
    -fomit-frame-pointer \
    -pipe \
	-std=c++17 \
    -pedantic -pedantic-errors \
    -Werror \
    -Weffc++ \
    -Wall -Wextra -Wshadow -Wdeprecated \
    -Wdiv-by-zero -Wfloat-equal -Wfloat-conversion -Wsign-compare -Wpointer-arith \
    -Wuninitialized -Wunreachable-code \
    -Wunused -Wunused-function -Wunused-label -Wunused-parameter -Wunused-but-set-parameter -Wunused-but-set-variable \
    -Wunused-value -Wunused-variable -Wunused-result \
    -Wmissing-field-initializers -Wmissing-format-attribute -Wmissing-include-dirs -Wmissing-noreturn")

# Add current build directory as include directory as it contains generated files.
#include_directories(SYSTEM ${CMAKE_BINARY_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)
add_executable(${PROJECT_NAME} src/clik.cpp)

################
# ur_rtde library
find_package(ur_rtde REQUIRED)
target_link_libraries(${PROJECT_NAME} PUBLIC ur_rtde::rtde)
#target_link_libraries(clik PUBLIC ur_rtde::rtde)

### pinocchio
find_package(pinocchio REQUIRED)
target_link_libraries(${PROJECT_NAME} PUBLIC pinocchio::pinocchio)
#target_link_libraries(clik PUBLIC pinocchio::pinocchio)

## pinocchio-gepetto-viewer
#find_package(pinocchio-gepetto-viewer REQUIRED)
#target_link_libraries(${PROJECT_NAME} PUBLIC pinocchio-gepetto-viewer)


#add_library(${PROJECT_NAME}-core OBJECT  ${CMAKE_CURRENT_SOURCE_DIR}/src/clik.cpp)
#set(LIBRARIES pinocchio::pinocchio ur_rtde::ur_rtde)
#set(LIBRARIES pinocchio ur_rtde)

# Create executable.
#add_executable(${PROJECT_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/src/${PROJECT_NAME}.cpp $<TARGET_OBJECTS:${PROJECT_NAME}-core>)
#add_executable(
#target_link_libraries(${PROJECT_NAME} ${LIBRARIES})
#target_link_libraries(${PROJECT_NAME} pinocchio::pinocchio)


# TODO this doesn't hurt right?
# Install executable.
#install(TARGETS ${PROJECT_NAME} DESTINATION bin COMPONENT ${PROJECT_NAME})

################################################################################
# TODO implement and then Enable unit testing.
#enable_testing()
#add_executable(${PROJECT_NAME}-runner ${CMAKE_CURRENT_SOURCE_DIR}/test/test-behavior.cpp $<TARGET_OBJECTS:${PROJECT_NAME}-core>)
#target_link_libraries(${PROJECT_NAME}-runner ${LIBRARIES})
#add_test(NAME ${PROJECT_NAME}-runner COMMAND ${PROJECT_NAME}-runner)


# TODO probably a good idea to start using doxygen or something similar
#option(INSTALL_DOCUMENTATION "Generate and install the documentation" OFF)
#set(DOXYGEN_USE_MATHJAX YES)
#set(DOXYGEN_USE_TEMPLATE_CSS YES)

# Create different building options
# TODO idk what this does actually
#option(ENABLE_VECTORIZATION
#       "Enable vectorization and further processor-related optimizations" OFF)
# TODO see how to do this with pybind11 (just extend ur_rtde approach)
#option(BUILD_PYTHON_INTERFACE "Build the python binding" ON)
#option(BUILD_BENCHMARK "Build the benchmark" ON)
#option(BUILD_EXAMPLES "Build the examples" ON)
#option(BUILD_WITH_IPOPT "Build the IPOPT wrapper." ON)
#option(GENERATE_PYTHON_STUBS
#       "Generate the Python stubs associated to the Python library" OFF)

#if(ENABLE_VECTORIZATION)
#  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")
#endif()

# Check if the conan file exist to find the dependencies
#if (EXISTS ${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
#include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
#conan_basic_setup(NO_OUTPUT_DIRS KEEP_RPATHS SKIP_STD TARGETS)
#set(Boost_USE_STATIC_LIBS ON)
#set(Boost_ADDITIONAL_VERSIONS 1.75.0)
#endif()
#find_package(Boost REQUIRED COMPONENTS system thread program_options)
#find_package(Threads)

############### pybind copied from ur_rtde and i don't get it ######

#if(${PYTHON_BINDINGS})
## This is required to find python site-packages for installation of ur_rtde python bindings.
#if(NOT PYBIND11_PYTHON_VERSION)
#	set(PYBIND11_PYTHON_VERSION "" CACHE STRING "Python version to use for compiling modules")
#endif()
#set(Python_ADDITIONAL_VERSIONS 3.11 3.10 3.9 3.8 3.7 3.6 3.5 3.4)
#find_package(PythonLibsNew ${PYBIND11_PYTHON_VERSION} REQUIRED)
#
#include(FetchContent)
#
#FetchContent_Declare(
#	pybind11-src
#	GIT_REPOSITORY https://github.com/pybind/pybind11.git
#	GIT_TAG        v2.10.1
#)
#FetchContent_MakeAvailable(pybind11-src)
#
#endif()
#
#
#if(${PYTHON_BINDINGS})
#pybind11_add_module(rtde_control SHARED ${LIB_SOURCE_FILES} ${LIB_HEADER_FILES} ${LIB_URCL_HEADER_FILES} src/rtde_python_bindings.cpp)
#target_include_directories(rtde_control
#		PUBLIC ${Boost_INCLUDE_DIRS}
#		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#		$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
#		$<INSTALL_INTERFACE:include>
#		PRIVATE ${PYBIND11_INCLUDE_DIR}  # from project CMakeLists.txt
#		PRIVATE ${pybind11_INCLUDE_DIR}  # from pybind11Config
#		PRIVATE ${PYTHON_INCLUDE_DIRS})
#target_link_libraries(rtde_control PRIVATE
#		#${PYTHON_LIBRARIES}
#		${Boost_SYSTEM_LIBRARY}
#		${Boost_THREAD_LIBRARY}
#		${CMAKE_THREAD_LIBS_INIT})
#		target_compile_definitions(rtde_control PRIVATE rtde_EXPORTS)
#
#pybind11_add_module(rtde_receive SHARED ${LIB_SOURCE_FILES} ${LIB_HEADER_FILES} src/rtde_python_bindings.cpp)
#target_include_directories(rtde_receive
#	PUBLIC ${Boost_INCLUDE_DIRS}
#	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#	$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
#	$<INSTALL_INTERFACE:include>
#	PRIVATE ${PYBIND11_INCLUDE_DIR}  # from project CMakeLists.txt
#	PRIVATE ${pybind11_INCLUDE_DIR}  # from pybind11Config
#	PRIVATE ${PYTHON_INCLUDE_DIRS})
#target_link_libraries(rtde_receive PRIVATE
#		#${PYTHON_LIBRARIES}
#		${Boost_SYSTEM_LIBRARY}
#		${Boost_THREAD_LIBRARY})
#		target_compile_definitions(rtde_receive PRIVATE rtde_EXPORTS)
#
#pybind11_add_module(rtde_io SHARED ${LIB_SOURCE_FILES} ${LIB_HEADER_FILES} src/rtde_python_bindings.cpp)
#target_include_directories(rtde_io
#		PUBLIC ${Boost_INCLUDE_DIRS}
#		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#		$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
#		$<INSTALL_INTERFACE:include>
#		PRIVATE ${PYBIND11_INCLUDE_DIR}  # from project CMakeLists.txt
#		PRIVATE ${pybind11_INCLUDE_DIR}  # from pybind11Config
#		PRIVATE ${PYTHON_INCLUDE_DIRS})
#target_link_libraries(rtde_io PRIVATE
#		#${PYTHON_LIBRARIES}
#		${Boost_SYSTEM_LIBRARY}
#		${Boost_THREAD_LIBRARY})
#target_compile_definitions(rtde_io PRIVATE rtde_EXPORTS)
#
#pybind11_add_module(script_client SHARED ${LIB_SOURCE_FILES} ${LIB_HEADER_FILES} src/rtde_python_bindings.cpp)
#target_include_directories(script_client
#		PUBLIC ${Boost_INCLUDE_DIRS}
#		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#		$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
#		$<INSTALL_INTERFACE:include>
#		PRIVATE ${PYBIND11_INCLUDE_DIR}  # from project CMakeLists.txt
#		PRIVATE ${pybind11_INCLUDE_DIR}  # from pybind11Config
#		PRIVATE ${PYTHON_INCLUDE_DIRS})
#	target_link_libraries(script_client PRIVATE
#		#${PYTHON_LIBRARIES}
#		${Boost_SYSTEM_LIBRARY}
#		${Boost_THREAD_LIBRARY})
#	target_compile_definitions(script_client PRIVATE rtde_EXPORTS)
#
#
#pybind11_add_module(dashboard_client SHARED ${LIB_SOURCE_FILES} ${LIB_HEADER_FILES} src/rtde_python_bindings.cpp)
#target_include_directories(dashboard_client
#	PUBLIC ${Boost_INCLUDE_DIRS}
#	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#	$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
#	$<INSTALL_INTERFACE:include>
#	PRIVATE ${PYBIND11_INCLUDE_DIR}  # from project CMakeLists.txt
#	PRIVATE ${pybind11_INCLUDE_DIR}  # from pybind11Config
#	PRIVATE ${PYTHON_INCLUDE_DIRS})
#target_link_libraries(dashboard_client PRIVATE
#	#${PYTHON_LIBRARIES}
#	${Boost_SYSTEM_LIBRARY}
#	${Boost_THREAD_LIBRARY})
#target_compile_definitions(dashboard_client PRIVATE rtde_EXPORTS)
#
#endif()

###################################################### 


#add_project_dependency(pinocchio 2.6.3 REQUIRED PKG_CONFIG_REQUIRES
#                       "pinocchio >= 2.6.3")
#if(BUILD_EXAMPLES
#   OR BUILD_TESTING
#   OR BUILD_BENCHMARK)
#  add_project_dependency(example-robot-data 4.0.7 REQUIRED PKG_CONFIG_REQUIRES
#                         "example-robot-data >= 4.0.7")
#else()
#  add_optional_dependency(example-robot-data)
#endif()

# i don't have experience with any of this,
# but it's probably good
#option(
#  BUILD_WITH_CODEGEN_SUPPORT
#  "Build the library with the Code Generation support (required CppADCodeGen)"
#  OFF)
#
#option(BUILD_WITH_MULTITHREADS
#       "Build the library with the Multithreading support (required OpenMP)"
#       OFF)
#if(BUILD_WITH_MULTITHREADS)
#  # If BUILD_WITH_NTHREADS defined, use the value - otherwise detect
#  if(NOT DEFINED BUILD_WITH_NTHREADS)
#    include(ProcessorCount)
#    ProcessorCount(NPROCESSOR)
#    if(NOT NPROCESSOR EQUAL 0)
#      math(EXPR NPROCESSOR ${NPROCESSOR}/2)
#      set(BUILD_WITH_NTHREADS
#          ${NPROCESSOR}
#          CACHE STRING "Number of threads")
#    else()
#      set(BUILD_WITH_NTHREADS
#          "4"
#          CACHE STRING "Number of threads") # Default setting
#    endif()
#  endif()
#  string(REGEX MATCH "^[0-9]+$" BUILD_WITH_NTHREADS ${BUILD_WITH_NTHREADS})
#  if(NOT BUILD_WITH_NTHREADS MATCHES "^[0-9]+$")
#    set(BUILD_WITH_NTHREADS 4) # Default setting 4.
#    message(
#      WARNING
#        "The number of threads has to be an integer value, set to ${BUILD_WITH_NTHREADS}"
#    )
#  endif()
#endif()
#
#if(BUILD_WITH_CODEGEN_SUPPORT)
#  add_project_dependency(cppad 20200000.0 REQUIRED)
#  add_project_dependency(cppadcg 2.4.1 REQUIRED)
#  add_definitions(-DCROCODDYL_WITH_CODEGEN)
#  # Pinocchio codegen related preproccessor defs.
#  add_definitions(-DPINOCCHIO_WITH_CPPADCG_SUPPORT)
#  add_definitions(-DPINOCCHIO_WITH_CPPAD_SUPPORT)
#  # Packaging for downstream.
#  set(PACKAGE_EXTRA_MACROS
#      "${PACKAGE_EXTRA_MACROS}\nADD_DEFINITIONS(-DCROCODDYL_WITH_CODEGEN -DPINOCCHIO_WITH_CPPAD_SUPPORT -DPINOCCHIO_WITH_CPPADCG_SUPPORT)"
#  )
#  if(NOT ${EIGEN3_VERSION} VERSION_GREATER "3.3.0")
#    add_definitions(-DPINOCCHIO_CPPAD_REQUIRES_MATRIX_BASE_PLUGIN)
#    set(PACKAGE_EXTRA_MACROS
#        "${PACKAGE_EXTRA_MACROS}\nADD_DEFINITIONS(-DPINOCCHIO_CPPAD_REQUIRES_MATRIX_BASE_PLUGIN)"
#    )
#  endif(NOT ${EIGEN3_VERSION} VERSION_GREATER "3.3.0")
#endif()
#
## Add OpenMP
#if(BUILD_WITH_MULTITHREADS)
#  find_package(OpenMP REQUIRED COMPONENTS CXX)
#
#  add_definitions(-DCROCODDYL_WITH_MULTITHREADING)
#  add_definitions(-DCROCODDYL_WITH_NTHREADS=${BUILD_WITH_NTHREADS})
#  set(PACKAGE_EXTRA_MACROS
#      "${PACKAGE_EXTRA_MACROS}\nADD_DEFINITIONS(-DCROCODDYL_WITH_MULTITHREADING -DCROCODDYL_WITH_NTHREADS=${BUILD_WITH_NTHREADS})"
#  )
#  set(OMP_NUM_THREADS ${BUILD_WITH_NTHREADS})
#endif()
#
## Add Ipopt
#if(BUILD_WITH_IPOPT AND IPOPT_FOUND)
#  add_definitions(-DCROCODDYL_WITH_IPOPT)
#  link_directories(${IPOPT_LIBRARY_DIRS})
#endif()

#set(BOOST_REQUIRED_COMPONENTS filesystem serialization system)
#set(BOOST_BUILD_COMPONENTS unit_test_framework)
#set_boost_default_options()
#export_boost_default_options()
#add_project_dependency(Boost 1.65 REQUIRED COMPONENTS
#                       ${BOOST_REQUIRED_COMPONENTS})
#find_package(Boost 1.65 REQUIRED COMPONENTS ${BOOST_BUILD_COMPONENTS})
#
#if(Boost_VERSION GREATER 107299)
#  # Silence a warning about a deprecated use of boost bind by boost python at
#  # least fo boost 1.73 to 1.75
#  add_definitions(-DBOOST_BIND_GLOBAL_PLACEHOLDERS)
#endif()

#target_include_directories(${PROJECT_NAME}
#					 PUBLIC $<INSTALL_INTERFACE:include>)
#target_link_libraries(${PROJECT_NAME} pinocchio::pinocchio)
#target_link_libraries(${PROJECT_NAME} Boost::filesystem Boost::system
#				Boost::serialization)

# add this if you add this 
#if(BUILD_WITH_MULTITHREADS)
#target_link_libraries(${PROJECT_NAME} OpenMP::OpenMP_CXX)
#endif()
#
#if(BUILD_WITH_IPOPT AND IPOPT_FOUND)
#target_link_libraries(${PROJECT_NAME} ipopt)
#target_include_directories(${PROJECT_NAME} PUBLIC ${IPOPT_INCLUDE_DIRS})
#endif()

# TODO try to figure it out
# Build the Python interface
#if(BUILD_PYTHON_INTERFACE)
#  # Include and add the bindings directory
#  include_directories(bindings)
#  add_subdirectory(bindings)
#endif(BUILD_PYTHON_INTERFACE)
#
## Build the unit tests
#if(BUILD_TESTING)
#  add_subdirectory(unittest)
#endif(BUILD_TESTING)
#
## Build the benchmark
#if(BUILD_BENCHMARK)
#  add_subdirectory(benchmark)
#endif(BUILD_BENCHMARK)
#
## Build the examples
#if(BUILD_EXAMPLES)
#  if(BUILD_PYTHON_INTERFACE)
#    add_subdirectory(examples)
#  else(BUILD_PYTHON_INTERFACE)
#    message(
#      WARNING "Python interface is not built, hence cannot build examples.")
#  endif(BUILD_PYTHON_INTERFACE)
#endif(BUILD_EXAMPLES)

# TODO: generate this as well for full points i guess
# PkgConfig packaging of the project
#pkg_config_append_libs(${PROJECT_NAME})
#pkg_config_append_boost_libs(${BOOST_REQUIRED_COMPONENTS})
