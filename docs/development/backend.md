# Backend

`Libuipc` has frontend and backend components. The frontend part is used to create the scene, objects, and geometries, and the backend part is used to simulate the physics of the scene.

A `libuipc` backend is a independent module that will be dynamically loaded and linked by the frontend. Typically througth the `engine::Engine` class.
The frontend user utilizes the backend functionalities through `World` interface.

```cpp
engine::Engine engine{"BACKEND_NAME"};
world::World world{engine};
```

## Create Your Own Backend

Say, you want to create a backend named `my_backend`. You need to create a new folder `my_backend/` in the `src/uipc/backends/` directory.

And then, you need to append a line of `add_subdirectory(my_backend)` in the `src/uipc/backends/CMakeLists.txt` file to include the new backend in the build system.

In the `my_backend/` folder, you need to create a `CMakeLists.txt` file to define the build rules for the backend.

```cmake
# my_backend/CMakeLists.txt
uipc_add_backend(my_backend)
file(GLOB SOURCES "*.cpp" "*.h" "details/*.inl")
target_sources(my_backend PRIVATE ${SOURCES})
```

The backend `none` is a good example for creating a new backend.

The entry point of the backend is the engine creation and destruction functions. You need to implement somewhere in the backend source code.

```cpp
#include <uipc/backends/common/module.h>

UIPC_BACKEND_API
UIPCEngineInterface* uipc_create_engine()
{
    // new your engine instance and return it
}

UIPC_BACKEND_API
void uipc_destroy_engine(UIPCEngineInterface* engine)
{
    // delete your engine instance
}
```

## Official Backends

`Libuipc` provides several official backends:

| Name                            | Description                                                                  |
| ------------------------------- | ---------------------------------------------------------------------------- |
| none                            | A dummy backend that does nothing, as a template for creating a new backend. |
| [cuda](./backend_cuda/index.md) | A backend that utilizes the GPU to compute the physics.                      |
| ...                             |                                                                              |

## Common Utilities

Some common utilities are shared among different backends. They are in the `src/uipc/backends/common/` directory. They are automatically included in the build system when you call `uipc_add_backend` in the `CMakeLists.txt` file.

It's recommended to use these utilities to make your backend engine more robust and maintainable.

### SimEngine

The `uipc::backend::SimEngine` class is the top-level class of the backend, this base class will help you to manage the backend simulation systems. You can derive your own engine class from it.

Call `build_systems()` will help you to build all the systems and their dependencies. Call `dump_system_infos()` will help you to dump the system information to the workspace of this backend.

```cpp
// my_backend/my_sim_engine.h
#include <uipc/backends/common/sim_engine.h>
namespace uipc::backend::my_backend
{
class UIPC_BACKEND_API MySimEngine : public SimEngine
{
  protected:
    void do_init(backend::WorldVisitor v) override;
    void do_advance() override;
    void do_sync() override;
    void do_retrieve() override;
    ...
};
}

// my_backend/my_sim_engine.cpp
namespace uipc::backend::my_backend
{
void MySimEngine::do_init(backend::WorldVisitor v)
{
    build_systems();
    dump_system_infos();
}
}
```

### SimSystem

The `uipc::backend::SimSystem` class is the base class of the backend simulation system. You can derive your own system class from it.

```cpp
// my_backend/my_sim_system.h
#include <uipc/backends/common/sim_system.h>
namespace uipc::backend::my_backend
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
namespace uipc::backend::my_backend
{
    REGISTER_SIM_SYSTEM(MySimSystem);
}
```

Call `REGISTER_SIM_SYSTEM(MySimSystem)` in source file to register your system to the backend engine automatically.

### Dependency

To build up the dependency between systems, you call the `require<T>` and `find<T>` in the `do_build` function of the system.

```cpp
// my_backend/my_sim_system.cpp
namespace uipc::backend::my_backend
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

`require<T>` will throw an exception if the system is not found, and `find<T>` will return a nullptr if the system is not found. If any dependency is not satisfied, the dependent system will throw an exception to invalidate itself.

You can also manually throw a `SimSystemException` to invalidate the system.

The backend common utilities will clean up the invalid systems and keep the valid systems in the backend engine. If such exception is upraised to the level of the backend engine, the engine will close itself and throw the exception to the frontend.

It's recommended to use the `reuiqre<T>` for those systems that are necessary for the current system to work properly, and use the `find<T>` for those systems that are optional.

To keep the reference of the other systems, you **should** use the `SimSystemSlot` and `SimSystemSlotCollection`. They will help you to manage the lifetime of the other systems properly. 

Some wierd situation may happen: the other system is valid when you require it, but it's invalid when you use it. The `SimSystemSlot` and `SimSystemSlotCollection` will help you to avoid such situation.

Or you should manually check the validity of the other systems before you use them.

### Lifecycle Functions

Lifecyle of a simulator is the most complex part of the backend, varying among different simulation methods. So the common utilities won't provide a default implementation for the lifecycle functions.

It's up to you to design the pipeline. I.e. how every `SimSystem` is updated in each phase of the simulation.

It's your responsibility to call the lifecycle functions in the engine's `do_init`, `do_advance`, `do_sync`, and `do_retrieve` functions.

But the common utilities still provide some basic tools.

#### SimAction

```cpp
#include <uipc/backends/common/sim_system.h>
#include <uipc/backends/common/sim_action_collection.h>

namespace uipc::backend::my_backend
{
class MyActionDispatcher : public SimSystem
{
    public:
    using SimSystem::SimSystem;
    void do_build() override{}
    void add_action(SimAction<void()>&& action);


    private:
    friend class MySimEngine;
    void dispatch_actions();
    SimActionCollection<void()> m_actions;
};
}

// my_backend/my_action_dispatcher.cpp
namespace uipc::backend::my_backend
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
// my_backend/my_sim_engine.h
#include <uipc/backends/common/sim_engine.h>
namespace uipc::backend::my_backend
{
// forward declaration
class MyActionDispatcher;

class UIPC_BACKEND_API MySimEngine : public SimEngine
{
  protected:
    void do_init(backend::WorldVisitor v) override;
    void do_advance() override;
    void do_sync() override;
    void do_retrieve() override;
   private:
    SimSystemSlot<MyActionDispatcher> m_action_dispatcher;
};
}

// my_backend/my_sim_engine.cpp
namespace uipc::backend::my_backend
{
void MySimEngine::do_init(backend::WorldVisitor v)
{
    build_systems();

    m_action_dispatcher = &require<MyActionDispatcher>();

    dump_system_infos();
}

void MySimEngine::do_advance()
{
    auto action_view = m_action_dispatcher->view();
    for (auto& action : action_view)
    {
        action();
    }
}
}
```

With the implementation above, other sim systems can add actions to the `MyActionDispatcher` in the `do_build` function. 

And because we can require the `MyActionDispatcher` in the `MySimEngine` and keep the reference of it, we can dispatch the actions in any where of the engine.

E.g. The Dispatcher can be a time to integrate some systems, or a time to solve the linear system, or a time to update the geometry.

#### Subsystem

Sometimes, it's a good idea to have a global system and several subsystems, all subsystem register itself to the global system, and the global system dispatch the lifecycle functions of the subsystems.

The common utilities don't provide a default implementation for you. But it's easy to implement it by yourself. Using the same idea of the `SimAction` and `SimActionCollection`.

Just replace the `SimAction` with `SimSystem`, `SimActionCollection` with `SimSystemSlotCollection`. And ask the subsystems to register themselves to the global system when `do_build`.

```cpp
// my_backend/my_global_system.h

namespace uipc::backend::my_backend
{
class MyGlobalSystem : public SimSystem
{
    public:
    using SimSystem::SimSystem;
    void do_build() override{}
    void add_subsystem(SimSystem* subsystem);
    void dispatch_subsystems();

    private:
    friend class MySimEngine;
    SimSystemSlotCollection<SimSystem> m_subsystems;
};
}

// my_backend/my_global_system.cpp
namespace uipc::backend::my_backend
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

Such global-subsystem pattern can be recursively used. A subsystem can also have its own subsystems.

