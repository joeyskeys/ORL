if(NOT GLFW_ROOT)
    if(WIN32)
        set(GLFW_ROOT "C:/Program Files" CACHE PATH "Location of GLFW")
    elseif(APPLE)
        set(GLFW_ROOT "/usr/local/Cellar" CACHE PATH "Location of GLFW")
    else()
        set(GLFW_ROOT "/usr/local" CACHE PATH "Location of GLFW")
    endif()
endif()

find_path(GLFW_INCLUDE_DIR GLFW/glfw3.h
    PATHS
        ${GLFW_ROOT}/include
        /usr/include)

find_library(GLFW_LIBRARY
    NAMES
        glfw
    PATHS
        ${GLFW_ROOT}/lib
        /usr/lib)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(GLFW
    REQUIRED_VARS
        GLFW_INCLUDE_DIR
        GLFW_LIBRARY)

if(GLFW_FOUND)
    set(GLFW_INCLUDE_DIRS ${GLFW_INCLUDE_DIR})
    set(GLFW_LIBRARIES ${GLFW_LIBRARY})
endif()