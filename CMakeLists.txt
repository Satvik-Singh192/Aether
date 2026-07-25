cmake_minimum_required(VERSION 3.20)

project(Aether LANGUAGES CXX)

set(CMAKE_POLICY_VERSION_MINIMUM 3.5)

set(CMAKE_CXX_STANDARD 20)

set(AETHER_BUILD_TARGET_DEFAULT "Desktop")
if(EMSCRIPTEN OR CMAKE_SYSTEM_NAME STREQUAL "Emscripten")
    set(AETHER_BUILD_TARGET_DEFAULT "WASM")
endif()

set(AETHER_BUILD_TARGET "${AETHER_BUILD_TARGET_DEFAULT}" CACHE STRING "Build target: Desktop or WASM")
set_property(CACHE AETHER_BUILD_TARGET PROPERTY STRINGS Desktop WASM)

include(FetchContent)

set(_aether_deps_root "${CMAKE_BINARY_DIR}/_deps")

function(aether_use_local_fetchcontent dependency_name source_dir_name)
    string(TOUPPER "${dependency_name}" dependency_name_upper)
    if(NOT DEFINED "FETCHCONTENT_SOURCE_DIR_${dependency_name_upper}")
        if(EXISTS "${_aether_deps_root}/${source_dir_name}/CMakeLists.txt")
            set("FETCHCONTENT_SOURCE_DIR_${dependency_name_upper}" "${_aether_deps_root}/${source_dir_name}" CACHE PATH "Local source dir for ${dependency_name}" FORCE)
        endif()
    endif()
endfunction()

aether_use_local_fetchcontent(glfw glfw-src)
aether_use_local_fetchcontent(glad glad-src)
aether_use_local_fetchcontent(glm glm-src)

FetchContent_Declare(
    glm
    GIT_REPOSITORY https://github.com/g-truc/glm.git
    GIT_TAG 0.9.9.8
)

FetchContent_MakeAvailable(glm)

file(GLOB_RECURSE AETHER_ENGINE_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/engine/*.cpp"
)

file(GLOB_RECURSE AETHER_RENDERER_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/renderer/*.cpp"
)

add_library(AetherEngine STATIC
    ${AETHER_ENGINE_SOURCES}
    engine/api/AetherAPI.hpp
    engine/api/RenderBody.hpp
)

target_include_directories(AetherEngine PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/engine
)

target_link_libraries(AetherEngine PRIVATE
    glm
)

if(AETHER_BUILD_TARGET STREQUAL "Desktop")
    aether_use_local_fetchcontent(imgui imgui-src)

    FetchContent_Declare(
        glfw
        GIT_REPOSITORY https://github.com/glfw/glfw.git
        GIT_TAG 3.3.8
    )

    FetchContent_MakeAvailable(glfw)

    FetchContent_Declare(
        glad
        GIT_REPOSITORY https://github.com/Dav1dde/glad.git
        GIT_TAG v0.1.36
    )

    FetchContent_Declare(
        imgui
        GIT_REPOSITORY https://github.com/ocornut/imgui.git
        GIT_TAG v1.90.8
    )

    FetchContent_MakeAvailable(imgui)

    FetchContent_MakeAvailable(glad)

    find_package(OpenGL REQUIRED)

    add_library(imgui STATIC
        ${imgui_SOURCE_DIR}/imgui.cpp
        ${imgui_SOURCE_DIR}/imgui_draw.cpp
        ${imgui_SOURCE_DIR}/imgui_tables.cpp
        ${imgui_SOURCE_DIR}/imgui_widgets.cpp
        ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
        ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
    )

    target_include_directories(imgui PUBLIC
        ${imgui_SOURCE_DIR}
        ${imgui_SOURCE_DIR}/backends
    )

    target_compile_definitions(imgui PRIVATE IMGUI_IMPL_OPENGL_LOADER_GLAD)

    target_link_libraries(imgui PRIVATE glfw OpenGL::GL)

    add_executable(AetherDesktop
        app/main.cpp
        app/test_scenarios.cpp
        ${AETHER_RENDERER_SOURCES}
    )

    target_include_directories(AetherDesktop PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/app
        ${CMAKE_CURRENT_SOURCE_DIR}/renderer
    )

    target_link_libraries(AetherDesktop PRIVATE
        AetherEngine
        glfw
        glad
        OpenGL::GL
        glm
        imgui
    )
elseif(AETHER_BUILD_TARGET STREQUAL "WASM")

    add_executable(AetherWasm
        wasm/main.cpp
    )

    target_link_libraries(AetherWasm PRIVATE
        AetherEngine
    )

    if(EMSCRIPTEN)
        target_link_options(AetherWasm PRIVATE
    "-sWASM=1"
    "-sMODULARIZE=1"
    "-sEXPORT_ES6=1"
    "-sALLOW_MEMORY_GROWTH=1"
    "-sENVIRONMENT=web"
    "-sEXPORTED_FUNCTIONS=['_Init','_Step']"
    "-sEXPORTED_RUNTIME_METHODS=['ccall']"
)
    endif()

else()
    message(FATAL_ERROR "Unknown AETHER_BUILD_TARGET value: ${AETHER_BUILD_TARGET}. Use Desktop or WASM.")
endif()