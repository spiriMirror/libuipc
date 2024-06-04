#include <app/test_common.h>
#include <uipc/common/timer.h>
#include <uipc/common/type_define.h>
#include <uipc_gui/common/platform_handle.h>

TEST_CASE("hello_bgfx", "[bgfx]")
{
    std::vector<bgfx::RendererType::Enum> supportedRenderers(8);
    auto n_renderer = bgfx::getSupportedRenderers(supportedRenderers.size(),
                                                  supportedRenderers.data());

    fmt::println("Supported renderers:");
    for(int i = 0; i < n_renderer; ++i)
    {
        auto renderer = supportedRenderers[i];
        auto name     = bgfx::getRendererName(renderer);
        fmt::println("({}) {}", i + 1, name);
    }

    glfwInit();

    uint32_t width  = 800;
    uint32_t height = 600;

    auto debug = BGFX_DEBUG_TEXT;
    auto reset = BGFX_RESET_VSYNC;
    GLFWwindow* window = glfwCreateWindow(width, height, "Hello, bgfx!", NULL, NULL);

    bgfx::PlatformData pd;
    bgfx::Init         init;
    init.type              = bgfx::RendererType::Count;
    init.vendorId          = BGFX_PCI_ID_NONE;
    init.platformData.nwh  = uipc::gui::glfw_native_window_handle(window);
    init.platformData.ndt  = uipc::gui::native_display_handle();
    init.platformData.type = uipc::gui::native_window_handle_type();
    init.resolution.width  = width;
    init.resolution.height = height;
    init.resolution.reset  = reset;
    bgfx::init(init);


    // Enable debug text.
    bgfx::setDebug(debug);

    // Set view 0 clear state.
    bgfx::setViewClear(0, BGFX_CLEAR_COLOR | BGFX_CLEAR_DEPTH, 0x303030ff, 1.0f, 0);

    uipc::Float time = 1.5e3;
    while(!glfwWindowShouldClose(window))
    {
        uipc::Timer timer{"loop"};
        glfwPollEvents();

        int width, height;
        glfwGetWindowSize(window, &width, &height);
        if(width != (int)width || height != (int)height)
        {
            width  = width;
            height = height;
            bgfx::reset(width, height, reset);
        }

        bgfx::setViewRect(0, 0, 0, uint16_t(width), uint16_t(height));
        bgfx::touch(0);

        bgfx::dbgTextClear();
        bgfx::dbgTextPrintf(0, 1, 0x4f, "Hello, bgfx! [This window will close in %f ms]", time);

        bgfx::frame();
        time -= timer.elapsed();
        if(time < 0)
            break;
    }

    uipc::GlobalTimer::current()->print_merged_timings();
    bgfx::shutdown();
    glfwTerminate();
}
