#include <uipc_gui/common/platform_handle.h>
#include <bgfx/bgfx.h>
#include <bx/bx.h>
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#if GLFW_VERSION_MINOR < 2
#error "GLFW 3.2 or later is required"
#endif  // GLFW_VERSION_MINOR < 2

#if BX_PLATFORM_LINUX
#if ENTRY_CONFIG_USE_WAYLAND
#include <wayland-egl.h>
#define GLFW_EXPOSE_NATIVE_WAYLAND
#else
#define GLFW_EXPOSE_NATIVE_X11
#define GLFW_EXPOSE_NATIVE_GLX
#endif
#elif BX_PLATFORM_OSX
#define GLFW_EXPOSE_NATIVE_COCOA
#define GLFW_EXPOSE_NATIVE_NSGL
#elif BX_PLATFORM_WINDOWS
#define GLFW_EXPOSE_NATIVE_WIN32
#define GLFW_EXPOSE_NATIVE_WGL
#endif  //
#include <GLFW/glfw3native.h>

namespace uipc::gui
{
void* glfw_native_window_handle(GLFWwindow* _window)
{
#if BX_PLATFORM_LINUX
#if ENTRY_CONFIG_USE_WAYLAND
    wl_egl_window* win_impl = (wl_egl_window*)glfwGetWindowUserPointer(_window);
    if(!win_impl)
    {
        int width, height;
        glfwGetWindowSize(_window, &width, &height);
        struct wl_surface* surface = (struct wl_surface*)glfwGetWaylandWindow(_window);
        if(!surface)
            return nullptr;
        win_impl = wl_egl_window_create(surface, width, height);
        glfwSetWindowUserPointer(_window, (void*)(uintptr_t)win_impl);
    }
    return (void*)(uintptr_t)win_impl;
#else
    return (void*)(uintptr_t)glfwGetX11Window(_window);
#endif
#elif BX_PLATFORM_OSX
    return glfwGetCocoaWindow(_window);
#elif BX_PLATFORM_WINDOWS
    return glfwGetWin32Window(_window);
#endif  // BX_PLATFORM_
}
bgfx::NativeWindowHandleType::Enum native_window_handle_type()
{
#if BX_PLATFORM_LINUX
#if ENTRY_CONFIG_USE_WAYLAND
    return bgfx::NativeWindowHandleType::Wayland;
#else
    return bgfx::NativeWindowHandleType::Default;
#endif  // ENTRY_CONFIG_USE_WAYLAND
#else
    return bgfx::NativeWindowHandleType::Default;
#endif  // BX_PLATFORM_*
}

void* native_display_handle()
{
#if BX_PLATFORM_LINUX
#if ENTRY_CONFIG_USE_WAYLAND
    return glfwGetWaylandDisplay();
#else
    return glfwGetX11Display();
#endif  // ENTRY_CONFIG_USE_WAYLAND
#else
    return NULL;
#endif  // BX_PLATFORM_*
}
}  // namespace uipc::gui
