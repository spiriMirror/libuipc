#pragma once
#include <GLFW/glfw3.h>
#include <bgfx/bgfx.h>
namespace uipc::gui
{
void* glfw_native_window_handle(GLFWwindow* window);

bgfx::NativeWindowHandleType::Enum native_window_handle_type();

void* native_display_handle();
}
