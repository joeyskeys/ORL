if(NOT GLM_ROOT)
    if(WIN32)
        set(GLM_ROOT "C:/Program Files" CACHE PATH "Location of GLM")
    elseif(APPLE)
        set(GLM_ROOT "/usr/local/Cellar" CACHE PATH "Location of GLM")
    else()
        set(GLM_ROOT "/usr/local" CACHE PATH "Location of GLM")
    endif()
endif()

find_path(GLM_INCLUDE_DIR glm/glm.hpp
    PATHS
        ${GLM_ROOT}/include
        /usr/include
        /usr/local/include)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(GLM
    REQUIRED_VARS
        GLM_INCLUDE_DIR)

if(GLM_FOUND)
    set(GLM_INCLUDE_DIRS ${GLM_INCLUDE_DIR})
endif()
