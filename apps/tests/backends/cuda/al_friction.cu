#include <app/app.h>
#include <contact_system/al_contact_function.h>
#include <contact_system/al_vertex_half_plane_contact_function.h>
#include <cuda_tool/cuda_tool.h>

namespace al_friction_test
{
using namespace uipc;
using namespace uipc::backend::cuda;
namespace cuda_tool = uipc::backend::cuda_tool;

__global__ void pt_energy_gradient_fd_kernel(cuda_tool::CBufferView<Vector3> previous,
                                             cuda_tool::CBufferView<Vector3> current,
                                             cuda_tool::BufferView<Float> result)
{
    using namespace sym::al_simplex_contact;

    constexpr Float friction_rate = 0.2;
    constexpr Float eps_vh        = 0.01;
    constexpr Float normal_force  = 2.0;
    constexpr Float h             = 1e-6;

    Vector12    gradient;
    Matrix12x12 hessian;
    PT_friction_gradient_hessian(gradient,
                                 hessian,
                                 friction_rate,
                                 eps_vh,
                                 normal_force,
                                 previous(0),
                                 previous(1),
                                 previous(2),
                                 previous(3),
                                 current(0),
                                 current(1),
                                 current(2),
                                 current(3));

    result(0) = PT_friction_energy(friction_rate,
                                   eps_vh,
                                   normal_force,
                                   previous(0),
                                   previous(1),
                                   previous(2),
                                   previous(3),
                                   current(0),
                                   current(1),
                                   current(2),
                                   current(3));

    for(int dof = 0; dof < 12; ++dof)
    {
        Vector3 perturbed[4] = {current(0), current(1), current(2), current(3)};
        perturbed[dof / 3](dof % 3) += h;
        const Float E_plus = PT_friction_energy(friction_rate,
                                                eps_vh,
                                                normal_force,
                                                previous(0),
                                                previous(1),
                                                previous(2),
                                                previous(3),
                                                perturbed[0],
                                                perturbed[1],
                                                perturbed[2],
                                                perturbed[3]);

        perturbed[dof / 3](dof % 3) -= 2.0 * h;
        const Float E_minus = PT_friction_energy(friction_rate,
                                                 eps_vh,
                                                 normal_force,
                                                 previous(0),
                                                 previous(1),
                                                 previous(2),
                                                 previous(3),
                                                 perturbed[0],
                                                 perturbed[1],
                                                 perturbed[2],
                                                 perturbed[3]);
        result(1 + dof)     = gradient(dof);
        result(13 + dof)    = (E_plus - E_minus) / (2.0 * h);
    }
}

__global__ void ee_energy_gradient_fd_kernel(cuda_tool::CBufferView<Vector3> previous,
                                             cuda_tool::CBufferView<Vector3> current,
                                             cuda_tool::BufferView<Float> result)
{
    using namespace sym::al_simplex_contact;

    constexpr Float friction_rate = 0.2;
    constexpr Float eps_vh        = 0.01;
    constexpr Float normal_force  = 2.0;
    constexpr Float h             = 1e-6;

    Vector12    gradient;
    Matrix12x12 hessian;
    EE_friction_gradient_hessian(gradient,
                                 hessian,
                                 friction_rate,
                                 eps_vh,
                                 normal_force,
                                 previous(0),
                                 previous(1),
                                 previous(2),
                                 previous(3),
                                 current(0),
                                 current(1),
                                 current(2),
                                 current(3));

    result(0) = EE_friction_energy(friction_rate,
                                   eps_vh,
                                   normal_force,
                                   previous(0),
                                   previous(1),
                                   previous(2),
                                   previous(3),
                                   current(0),
                                   current(1),
                                   current(2),
                                   current(3));

    for(int dof = 0; dof < 12; ++dof)
    {
        Vector3 perturbed[4] = {current(0), current(1), current(2), current(3)};
        perturbed[dof / 3](dof % 3) += h;
        const Float E_plus = EE_friction_energy(friction_rate,
                                                eps_vh,
                                                normal_force,
                                                previous(0),
                                                previous(1),
                                                previous(2),
                                                previous(3),
                                                perturbed[0],
                                                perturbed[1],
                                                perturbed[2],
                                                perturbed[3]);

        perturbed[dof / 3](dof % 3) -= 2.0 * h;
        const Float E_minus = EE_friction_energy(friction_rate,
                                                 eps_vh,
                                                 normal_force,
                                                 previous(0),
                                                 previous(1),
                                                 previous(2),
                                                 previous(3),
                                                 perturbed[0],
                                                 perturbed[1],
                                                 perturbed[2],
                                                 perturbed[3]);
        result(1 + dof)     = gradient(dof);
        result(13 + dof)    = (E_plus - E_minus) / (2.0 * h);
    }

    result(25) = EE_friction_mollified(previous(0),
                                       previous(1),
                                       previous(2),
                                       previous(3),
                                       previous(0),
                                       previous(1),
                                       previous(2),
                                       previous(3));
    const Vector3 parallel_0{-1.0, 0.0, 0.0};
    const Vector3 parallel_1{1.0, 0.0, 0.0};
    const Vector3 parallel_2{-1.0, 0.0, 0.2};
    const Vector3 parallel_3{1.0, 0.0, 0.2};
    result(26) = EE_friction_mollified(
        parallel_0, parallel_1, parallel_2, parallel_3, parallel_0, parallel_1, parallel_2, parallel_3);
}

__global__ void ph_energy_gradient_fd_kernel(Vector3 previous,
                                             Vector3 current,
                                             Vector3 normal,
                                             cuda_tool::BufferView<Float> result)
{
    using namespace sym::al_vertex_half_plane_contact;

    constexpr Float friction_rate = 0.2;
    constexpr Float eps_vh        = 0.01;
    constexpr Float normal_force  = 2.0;
    constexpr Float h             = 1e-6;

    Vector3   gradient;
    Matrix3x3 hessian;
    half_plane_frictional_gradient_hessian(
        gradient, hessian, friction_rate, eps_vh, normal_force, current, previous, normal);
    result(0) = half_plane_frictional_energy(
        friction_rate, eps_vh, normal_force, current, previous, normal);

    for(int dof = 0; dof < 3; ++dof)
    {
        Vector3 perturbed = current;
        perturbed(dof) += h;
        const Float E_plus = half_plane_frictional_energy(
            friction_rate, eps_vh, normal_force, perturbed, previous, normal);
        perturbed(dof) -= 2.0 * h;
        const Float E_minus = half_plane_frictional_energy(
            friction_rate, eps_vh, normal_force, perturbed, previous, normal);
        result(1 + dof) = gradient(dof);
        result(4 + dof) = (E_plus - E_minus) / (2.0 * h);
    }
}
}  // namespace al_friction_test

TEST_CASE("AL point-triangle friction gradient matches its energy", "[cuda][al-ipc][friction]")
{
    using namespace uipc;
    namespace cuda_tool = uipc::backend::cuda_tool;

    cuda_tool::DeviceVector<Vector3> previous{Vector3{0.2, 0.3, 0.4},
                                              Vector3{-1.0, -1.0, 0.0},
                                              Vector3{1.0, -1.0, 0.0},
                                              Vector3{0.0, 1.0, 0.0}};
    cuda_tool::DeviceVector<Vector3> current{Vector3{0.24, 0.28, 0.41},
                                             Vector3{-0.98, -1.01, 0.0},
                                             Vector3{1.03, -0.99, 0.01},
                                             Vector3{0.01, 1.02, -0.01}};
    cuda_tool::DeviceVector<Float>   result(25);

    al_friction_test::pt_energy_gradient_fd_kernel<<<1, 1>>>(
        previous.cview(), current.cview(), result.view());
    CUDA_TOOL_CHECK(cudaGetLastError());

    std::vector<Float> host;
    result.copy_to(host);
    CHECK(std::isfinite(host[0]));
    for(int dof = 0; dof < 12; ++dof)
    {
        CAPTURE(dof, host[1 + dof], host[13 + dof]);
        CHECK(host[1 + dof] == Catch::Approx(host[13 + dof]).epsilon(1e-6).margin(1e-8));
    }
}

TEST_CASE("AL edge-edge friction gradient matches its energy", "[cuda][al-ipc][friction]")
{
    using namespace uipc;
    namespace cuda_tool = uipc::backend::cuda_tool;

    cuda_tool::DeviceVector<Vector3> previous{Vector3{-1.0, 0.0, 0.0},
                                              Vector3{1.0, 0.0, 0.0},
                                              Vector3{0.0, -1.0, 0.3},
                                              Vector3{0.0, 1.0, 0.3}};
    cuda_tool::DeviceVector<Vector3> current{Vector3{-0.97, 0.02, 0.01},
                                             Vector3{1.04, -0.01, -0.02},
                                             Vector3{-0.02, -0.96, 0.31},
                                             Vector3{0.01, 1.03, 0.28}};
    cuda_tool::DeviceVector<Float>   result(27);

    al_friction_test::ee_energy_gradient_fd_kernel<<<1, 1>>>(
        previous.cview(), current.cview(), result.view());
    CUDA_TOOL_CHECK(cudaGetLastError());

    std::vector<Float> host;
    result.copy_to(host);
    CHECK(std::isfinite(host[0]));
    for(int dof = 0; dof < 12; ++dof)
    {
        CAPTURE(dof, host[1 + dof], host[13 + dof]);
        CHECK(host[1 + dof] == Catch::Approx(host[13 + dof]).epsilon(1e-6).margin(1e-8));
    }
    CHECK(host[25] == 0.0);
    CHECK(host[26] == 1.0);
}

TEST_CASE("AL point-half-plane friction gradient matches its energy", "[cuda][al-ipc][friction]")
{
    using namespace uipc;
    namespace cuda_tool = uipc::backend::cuda_tool;

    cuda_tool::DeviceVector<Float> result(7);
    al_friction_test::ph_energy_gradient_fd_kernel<<<1, 1>>>(Vector3{0.1, 0.2, 0.3},
                                                             Vector3{0.14, 0.21, 0.28},
                                                             Vector3{0.0, 1.0, 0.0},
                                                             result.view());
    CUDA_TOOL_CHECK(cudaGetLastError());

    std::vector<Float> host;
    result.copy_to(host);
    CHECK(std::isfinite(host[0]));
    for(int dof = 0; dof < 3; ++dof)
    {
        CAPTURE(dof, host[1 + dof], host[4 + dof]);
        CHECK(host[1 + dof] == Catch::Approx(host[4 + dof]).epsilon(1e-6).margin(1e-8));
    }
}
