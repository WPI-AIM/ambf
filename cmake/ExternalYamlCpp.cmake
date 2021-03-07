include(ExternalProject)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(YAML_CPP_ROOT ${CMAKE_BINARY_DIR}/external/yaml_cpp)
set(YAML_CPP_LIB_DIR ${YAML_CPP_ROOT}/bin/lib)
set(YAML_CPP_INCLUDE_DIR ${YAML_CPP_ROOT}/bin/include)

ExternalProject_Add(YAML_CPP_EXTERNAL_PROJECT
                    PREFIX ${YAML_CPP_ROOT}
                    GIT_REPOSITORY "https://github.com/jbeder/yaml-cpp.git"
                    GIT_TAG "master"
                    UPDATE_COMMAND ""
                    PATCH_COMMAND ""
                    BINARY_DIR ${YAML_CPP_ROOT}/src/yaml_cpp
                    SOURCE_DIR ${YAML_CPP_ROOT}/src/yaml_cpp
                    INSTALL_DIR ${YAML_CPP_ROOT}/bin
                    BUILD_ALWAYS False
                    CMAKE_ARGS -DYAML_CPP_BUILD_TESTS=OFF -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>
                    BUILD_COMMAND make
                    )

#add_library(yaml-cpp UNKNOWN IMPORTED)
#set_target_properties(yaml-cpp PROPERTIES IMPORTED_LOCATION ${YAML_CPP_LIB_DIR}/libyaml-cpp.a)

# message("%%% YAML CPP INCLUDES: " ${YAML_CPP_INCLUDE_DIR})
# message("%%% YAML CPP LIBRARY DIR: " ${YAML_CPP_LIB_DIR})

link_directories(${YAML_CPP_LIB_DIR})
set(YAML_CPP_LIBRARIES yaml-cpp)
