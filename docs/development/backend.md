# Backend

`Libuipc` has frontend and backend components. The frontend is used to create the scene, objects, and geometries; the backend simulates the physics of the scene.

A `libuipc` backend is an independent module (a dynamic library named `uipc_backend_<name>`) that is dynamically loaded by the frontend when an `Engine` is created. The frontend user drives the backend through the `World` interface:

```cpp
core::Engine engine{"BACKEND_NAME", "workspace/path"};
core::World  world{engine};
```

## Backend Module ABI

A backend module must export exactly three C symbols (declared in `src/backends/common/module.h`):

```cpp
extern "C"
{
    // 1. Called once right after the dynamic library is loaded.
    //    Synchronizes the host's std::pmr::memory_resource into the module,
    //    so pmr containers allocate consistently across the DLL boundary.
    UIPC_BACKEND_API void uipc_init_module(UIPCModuleInitInfo* info);

    // 2. Called to create an engine instance.
    UIPC_BACKEND_API core::IEngine* uipc_create_engine(backend::EngineCreateInfo* info);

    // 3. Called to destroy the engine instance created by uipc_create_engine.
    UIPC_BACKEND_API void uipc_destroy_engine(core::IEngine* engine);
}
```

- `UIPCModuleInitInfo` (`include/uipc/backend/module_init_info.h`) carries the module name and the host PMR memory resource. The shared implementation in `src/backends/common/module.cpp` does `std::pmr::set_default_resource(info->memory_resource)` — reuse it.
- `EngineCreateInfo` (`include/uipc/backend/engine_create_info.h`) carries the absolute `workspace` path and the engine `Json` config (e.g. `config["gpu"]["device"]`).
- The frontend resolves the module from `uipc::config()["module_dir"]`, loading `uipc_backend_<name>` (with the platform shared-library extension). Loaded modules are cached, so repeated `Engine{"cuda"}` constructions reuse the same library.

## Tutorial: A Minimal Backend in 30 Minutes

The fastest way to create a backend is to copy the `none` backend (`src/backends/none/`), which implements the full skeleton with no-op physics.

### Step 1: Create the directory and build rules

Create `src/backends/my_backend/` and register it in `src/backends/CMakeLists.txt`:

```cmake
# src/backends/my_backend/CMakeLists.txt
uipc_add_backend(my_backend)
file(GLOB SOURCES "*.cpp" "*.h" "details/*.inl")
target_sources(my_backend PRIVATE ${SOURCES})
```

`uipc_add_backend` (defined in `src/backends/CMakeLists.txt`) creates a MODULE library target named `uipc_backend_my_backend`, links the backend common utilities, and sets the output directory so the frontend can find it.

### Step 2: Implement the entrance

```cpp
// src/backends/my_backend/entrance.cpp
#include <backends/common/module.h>
#include <my_backend/sim_engine.h>

using namespace uipc;

extern "C" UIPC_BACKEND_API core::IEngine* uipc_create_engine(
    backend::EngineCreateInfo* info)
{
    return new my_backend::SimEngine(*info);
}

extern "C" UIPC_BACKEND_API void uipc_destroy_engine(core::IEngine* engine)
{
    delete engine;
}
```

(`uipc_init_module` is already provided by `src/backends/common/module.cpp` — do not redefine it.)

### Step 3: Implement the engine

Derive from `uipc::backend::SimEngine` (`src/backends/common/sim_engine.h`) and override the lifecycle hooks of `core::IEngine`:

```cpp
// src/backends/my_backend/sim_engine.h
#pragma once
#include <backends/common/sim_engine.h>

namespace uipc::my_backend
{
class SimEngine final : public backend::SimEngine
{
  public:
    using backend::SimEngine::SimEngine;

  private:
    void do_init(backend::WorldVisitor v) override;
    void do_advance() override;
    void do_sync() override;
    void do_retrieve() override;
    // optional: do_dump / do_recover / do_to_json / get_frame / get_status / get_features
};
}  // namespace uipc::my_backend
```

```cpp
// src/backends/my_backend/sim_engine.cpp
#include <my_backend/sim_engine.h>

namespace uipc::my_backend
{
void SimEngine::do_init(backend::WorldVisitor v)
{
    // 1. pull scene data you need from the visitor
    // 2. build all registered sim systems and their dependencies
    build_systems();
    // 3. dump system topology to <workspace>/systems.json for debugging
    dump_system_infos();
}

void SimEngine::do_advance()
{
    // drive your simulation pipeline here
}

void SimEngine::do_sync()    { /* upload results to the frontend buffers */ }
void SimEngine::do_retrieve() { /* write back geometry attributes */ }
}  // namespace uipc::my_backend
```

### Step 4: Build and run

```bash
cmake -S . -B build -DUIPC_WITH_CUDA_BACKEND=OFF
cmake --build build --config Release -j8
```

Then create the engine with your backend name:

```cpp
core::Engine engine{"my_backend", "my_workspace"};
```

If the module fails to load, check that `uipc_backend_my_backend.{dll,so}` sits next to the other backend modules (or in `config["module_dir"]`).

## Official Backends

| Name                            | Description                                                                  |
| ------------------------------- | ---------------------------------------------------------------------------- |
| none                            | A dummy backend that does nothing, as a template for creating a new backend. |
| [cuda](./backend_cuda/index.md) | A backend that utilizes the GPU to compute the physics.                      |

## Common Utilities

Some common utilities are shared among different backends. They are in `src/backends/common/` and are automatically linked when you call `uipc_add_backend`.

It's recommended to use these utilities to make your backend engine more robust and maintainable.

### SimEngine

The `uipc::backend::SimEngine` class is the top-level class of the backend; this base class helps you manage the backend simulation systems. Derive your own engine class from it.

Call `build_systems()` to build all the systems and their dependencies. Call `dump_system_infos()` to dump the system information to the workspace of this backend.

### SimSystem

The `uipc::backend::SimSystem` class is the base class of a backend simulation system. Derive your own system class from it.

```cpp
// my_backend/my_sim_system.h
#include <backends/common/sim_system.h>
namespace uipc::my_backend
{
    class MySimSystem : public backend::SimSystem
    {
      public:
        using SimSystem::SimSystem;
      protected:
        void do_build() override;
        // a safe way to keep the reference of other system
        SimSystemSlot<OtherSimSystem> other_system;
        // a safe way to keep the reference of a collection of other systems
        SimSystemSlotCollection<AnotherSimSystem> another_systems;
    };
}

// my_backend/my_sim_system.cpp
namespace uipc::my_backend
{
    REGISTER_SIM_SYSTEM(MySimSystem);
}
```

Call `REGISTER_SIM_SYSTEM(MySimSystem)` in the source file to register your system to the backend engine automatically. Note that the constructor signature determines which engine the system belongs to: a system taking `cuda::SimEngine&` is only instantiated by the CUDA engine, a system taking `backend::SimEngine&` is instantiated by any engine.

### Dependency

To build up the dependency between systems, call `require<T>` and `find<T>` in the `do_build` function of the system.

```cpp
namespace uipc::my_backend
{
void MySimSystem::do_build()
{
    // require other systems
    auto& other_system_ref = require<OtherSimSystem>();
    other_system.register_system(&other_system_ref);

    // find other systems
    auto* another_system_ptr = find<AnotherSimSystem>();
    another_systems.register_system(another_system_ptr);

    if(...) // if some bad condition happened
        throw SimSystemException("This system is invalid");
}
}
```

`require<T>` will throw an exception if the system is not found, and `find<T>` will return a nullptr if the system is not found. If any dependency is not satisfied, the dependent system should throw to invalidate itself. You can also manually throw a `SimSystemException` to invalidate the system.

The backend common utilities will clean up the invalid systems — including any system that **strongly depends** on an invalid one (weak dependencies are tolerated) — and keep the valid systems in the backend engine. If such an exception propagates to the level of the backend engine, the engine will close itself and throw the exception to the frontend.

It's recommended to use `require<T>` for systems that are necessary, and `find<T>` for optional ones. By default `find<T>` matches the exact type only (`{.exact = true}`); pass `find<T>({.exact = false})` to also accept a **derived** implementation of `T`, which is how backends override a base system's functionality with a specialized one.

To keep references to other systems, you **should** use `SimSystemSlot` and `SimSystemSlotCollection`. They manage the lifetime of the referenced systems properly: a system may be valid when you require it but invalid later (e.g. after a cascading cleanup), and the slots guard against such use-after-invalidation.

### Lifecycle Functions

The lifecycle of a simulator is the most complex part of a backend, varying among different simulation methods. So the common utilities won't provide a default implementation for the lifecycle functions.

It's up to you to design the pipeline, i.e. how every `SimSystem` is updated in each phase of the simulation. It's your responsibility to call the lifecycle functions in the engine's `do_init`, `do_advance`, `do_sync`, and `do_retrieve` functions.

But the common utilities still provide some basic tools.

#### SimAction

```cpp
#include <backends/common/sim_system.h>
#include <backends/common/sim_action_collection.h>

namespace uipc::my_backend
{
class MyActionDispatcher : public SimSystem
{
  public:
    using SimSystem::SimSystem;
    void do_build() override {}
    void add_action(SimAction<void()>&& action);

  private:
    friend class SimEngine;
    void dispatch_actions();
    SimActionCollection<void()> m_actions;
};
}

// my_backend/my_action_dispatcher.cpp
namespace uipc::my_backend
{
REGISTER_SIM_SYSTEM(MyActionDispatcher);

void MyActionDispatcher::add_action(SimAction<void()>&& action)
{
    m_actions.add_action(std::move(action));
}

void MyActionDispatcher::dispatch_actions()
{
    for(auto& action : m_actions.view())
    {
        action();
    }
}
}
```

```cpp
// my_backend/sim_engine.cpp
namespace uipc::my_backend
{
void SimEngine::do_init(backend::WorldVisitor v)
{
    build_systems();

    m_action_dispatcher = &require<MyActionDispatcher>();

    dump_system_infos();
}

void SimEngine::do_advance()
{
    m_action_dispatcher->dispatch_actions();
}
}
```

With the implementation above, other sim systems can add actions to the `MyActionDispatcher` in their `do_build` function. And because the engine can require the `MyActionDispatcher` and keep a reference to it, we can dispatch the actions anywhere in the engine.

E.g. the dispatcher can represent a time to integrate some systems, or a time to solve the linear system, or a time to update the geometry. The CUDA backend uses exactly this pattern with `on_init_scene / on_rebuild_scene / on_write_scene` events (see `src/backends/cuda/sim_system.h`).

#### Subsystem

Sometimes, it's a good idea to have a global system and several subsystems; all subsystems register themselves to the global system, and the global system dispatches the lifecycle functions of the subsystems.

The common utilities don't provide a default implementation, but it's easy to build one using the same idea as `SimAction` and `SimActionCollection`: just replace `SimAction` with `SimSystem` and `SimActionCollection` with `SimSystemSlotCollection`, and ask the subsystems to register themselves to the global system in `do_build`.

```cpp
// my_backend/my_global_system.h
namespace uipc::my_backend
{
class MyGlobalSystem : public SimSystem
{
  public:
    using SimSystem::SimSystem;
    void do_build() override {}
    void add_subsystem(SimSystem* subsystem);
    void dispatch_subsystems();

  private:
    friend class SimEngine;
    SimSystemSlotCollection<SimSystem> m_subsystems;
};
}

// my_backend/my_global_system.cpp
namespace uipc::my_backend
{
REGISTER_SIM_SYSTEM(MyGlobalSystem);

void MyGlobalSystem::add_subsystem(SimSystem* subsystem)
{
    m_subsystems.register_system(subsystem);
}

void MyGlobalSystem::dispatch_subsystems()
{
    for(auto& subsystem : m_subsystems.view())
    {
        subsystem->advance();
    }
}
}
```

Of course, you should let the engine require the global system and dispatch the subsystems in the lifecycle functions.

Such a global-subsystem pattern can be used recursively: a subsystem can also have its own subsystems.

## Further Reading

- [Development Overview](./index.md) — the Data-Oriented Programming and Reporter-Manager-Receiver (RMR) design.
- [CUDA Backend](./backend_cuda/index.md) — a full-featured production backend.
- [Deterministic Mode](./deterministic_mode.md) — reproducibility design notes for backend authors.
