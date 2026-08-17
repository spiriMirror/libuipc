# 02 — 核心架构（core 模块与后端插件机制）

## 三层结构

```
core::Engine / World / Scene          ← 公开句柄层（仅持 S<internal::X>，纯转发）
  → core::internal::Engine / World / Scene   ← 生命周期与逻辑层
    → backend::IEngine（虚接口）       ← 后端动态库实现
```

- 公开层：`include/uipc/core/{engine,world,scene}.h`，实现 `src/core/core/{engine,world,scene}.cpp`
- internal 层：`include/uipc/core/internal/*.h` + `src/core/core/internal/*.cpp`
- 后端接口：`include/uipc/core/i_engine.h`

## 后端模块加载（插件 ABI）

`src/core/core/internal/engine.cpp` 的 `Engine::Impl::load_module(backend_name)`：

1. 用 `dylib` 从 `uipc::config()["module_dir"]` 加载 `uipc_backend_<name>` 动态库（静态缓存 `m_cache` + 互斥锁，同名模块只加载一次）。
2. 调用导出符号 **`uipc_init_module(UIPCModuleInitInfo*)`**（定义见 `include/uipc/backend/module_init_info.h`）：把 host 的 `std::pmr::get_default_resource()` 传给后端并 `set_default_resource`，保证跨 DLL 的 pmr 容器分配器一致。
3. 调 **`uipc_create_engine(EngineCreateInfo*)`** 创建 `IEngine*`；**`uipc_destroy_engine`** 作 deleter。`EngineCreateInfo{ workspace, Json config }`（`include/uipc/backend/engine_create_info.h`）。
4. 三个导出符号声明在 `src/backends/common/module.h`；各后端在 `entrance.cpp` 实现（cuda: `new cuda::SimEngine(info)`；none: `new none::SimEngine(info)`）。

也支持注入自定义引擎：`Engine(S<IEngine> overrider)` 构造（pyuipc 等场景用），不加载模块。

其他细节：
- workspace 构造时取绝对路径，不存在则创建。
- `advance()` 清除 sync 标志；`retrieve()` 若未 sync 先自动 `sync()`。
- `Engine::default_config()` 可被环境变量 `UIPC_ENGINE_DEFAULT_CONFIG`（指向 JSON 文件）覆盖；默认 `gpu/device=0`、gui enable。
- 每次后端调用包 `LogPatternGuard{backend_name}` 切换日志前缀。

## IEngine 模板方法

`include/uipc/core/i_engine.h`：公开非虚 `init/advance/sync/retrieve/to_json/dump/recover/frame/status/features/insert_sanity_checkers` → 受保护纯虚 `do_init/do_advance/do_sync/do_retrieve/get_frame/get_status/get_features`；`do_to_json/do_dump/do_recover/do_insert_sanity_checkers` 有空默认实现。

## World 生命周期（`src/core/core/internal/world.cpp`）

持有 `S<internal::Scene>`、`W<internal::Engine>`（weak 防循环引用）、`S<SanityChecker>`、`m_valid`。

**`World::init(Scene&)` 四步**：
1. 记录 scene（已 init 则直接返回）；
2. config 中 `sanity_check/enable` 为真时创建 `SanityChecker` 执行检查，失败则 `m_valid=false` 且跳过 init；
3. `m_scene->init(*this)`；
4. `engine->init(*this)`；之后检查 `engine->status().has_error()`，有错则 world 失效。

`advance/sync/retrieve/dump`：每步先查 `m_valid`，调用后查 `EngineStatusCollection::has_error()`，出错则 world **永久失效**并 log error。
`recover(aim_frame)`：成功后若 `diff_sim.parameters().size() > 0` 则 `parameters().broadcast()`。

## internal::Scene（`src/core/core/internal/scene.cpp`）

- `init(world)`：记录 world 裸指针；`m_constitution_tabular.init(*this)` 扫描全部几何 meta 收集 UID；`diff_sim/enable` 时 `m_diff_sim.init(*this)`；置 `m_started=true`。
- **pending 机制**：world 启动后几何增删不立即生效，进 `m_pending_create/m_pending_destroy`（`GeometryCollection`），由 `solve_pending()` 统一结算。`Scene::Objects::destroy` 按 `is_started()/is_pending()` 选直接 destroy 或 pending_destroy。
- `dt()` 从 config 属性 `"dt"` 读取。
- `update_from(const SceneSnapshotCommit&)`：应用增量 commit 更新 config/objects/contact_tabular/geometries/rest_geometries。

## Scene 默认配置（`src/core/core/scene_default_config.cpp`）

用 `AttributeCollection`（resize(1)）存配置。关键键：
- `dt=0.01`、`gravity`、`cfl/enable`
- `integrator/type="bdf1"`
- `newton/*`（max_iter、velocity_tol 用单位字面量 `0.05_m/1.0_s`）
- `linear_system/solver="fused_pcg"`
- `line_search/*`
- `contact/enable`、`contact/d_hat`、`contact/constitution="ipc"`（或 `"al-ipc"` 及其调参）
- `collision_detection/*`、`sanity_check/*`、`diff_sim/enable`

完整键表见 `docs/specification/scene_config.md`。

## RMR（Reporter-Manager-Receiver）与 SimSystem 体系

定义在 `src/backends/common/`，类层次：

```
core::IEngine
 └── backend::SimEngine            (common/sim_engine.h)
      ├── cuda::SimEngine          (cuda/sim_engine.h + engine/*.cu)
      └── none::SimEngine          (none/)

backend::ISimSystem                 (common/i_sim_system.h)
 └── backend::SimSystem            (common/sim_system.h)
      └── cuda::SimSystem          (cuda/sim_system.h)
           ├── Manager（GlobalVertexManager、GlobalContactManager、GlobalLinearSystem…）
           ├── Reporter / Subsystem（Animator、LineSearchReporter、DiagLinearSubsystem…）
           └── Receiver（DyTopoEffectReceiver、ContactReceiver…）
```

要点：
- **`ISimSystem`**：`name/is_valid/is_engine_aware/is_building`、强/弱依赖（`strong_dependencies/weak_dependencies`）、`to_json`；dump/recover 生命周期 `dump/try_recover/apply_recover/clear_recover` 对应 `do_*` 虚函数；`BaseInfo{frame, workspace, config}`。
- **`backend::SimSystem`**：持 `SimEngine&`；`find<T>(QueryOptions)/require<T>` 在 `SimSystemCollection` 查其它系统；`require` 失败抛 `SimSystemException` → 本系统 build 期判 invalid。
- **`cuda::SimSystem`**：新增事件注册 `on_init_scene/on_rebuild_scene/on_write_scene`（只能在 `do_build()` 中调用，动作存入引擎 `SimActionCollection`）；`check_state(SimEngineState)` 断言管线阶段。
- **`SimSystemCollection`**（`common/sim_system_collection.cpp`）：key 为 `typeid(s).hash_code()`；`build_systems()` 先统一 `set_building(true)` → 逐个 `build()`（内调 `do_build()`）→ 捕获异常标 invalid → `cleanup_invalid_systems()` **级联删除强依赖失效者**（弱依赖不影响）。
- **覆写机制**：`QueryOptions{exact}`；`exact=false` 允许返回派生类型系统 —— 即用派生类替换基类功能。另有 `*FeatureOverrider`（如 `affine_body_state_accessor_feature.cu`）通过 `engine.features()` 覆盖 feature。
- **自动注册**：`REGISTER_SIM_SYSTEM(T)` 宏（`common/sim_system.h` 末尾）借助 `SimSystemAutoRegister` 静态期注册 creator；`SimEngine::build_systems()` 遍历 creator 实例化。creator 构造参数类型决定归属引擎（cuda 系统构造参数是 `cuda::SimEngine&`，dynamic_cast 校验）。
- **SimSystemSlot<T> / SimSystemSlotCollection<T>**（`common/sim_system_slot.h`）：惰性绑定插槽；`register_sim_system(T&)` 挂入 manager 的 slot；`lazy_init()` 首次访问时确认绑定。Manager 用 SlotCollection 收集全部 reporter/subsystem。

## 周边子系统（core 内）

- **SanityCheck**（`src/sanity_check/`）：init 前场景校验（表面距离/交叉、半平面距离等内置检查器），测试见 `apps/tests/sanity_check/`。
- **DiffSim**（`include/uipc/diff_sim/`）：参数集合 `ParameterCollection`，`broadcast()` 在 recover 后触发；GPU 侧在 `src/backends/cuda/diff_sim/`。
- **SceneSnapshot/Commit**：场景增量序列化（`SceneIO` 的 `commit/update`），用于 GUI/网络同步。

## 日志、错误与基础设施（`include/uipc/common/`）

- `Logger`（spdlog 封装）、`Timer`（作用域计时，嵌套成树，输出 `timer_frames.json`）、`Json`（nlohmann）、`unit.h`（`1.0_GPa` 等字面量）、`UIPC_ASSERT` / `UIPC_ASSERT_THROW`（fast-fail 约定：内部不变量用前者，用户输入用后者）、`Exception`、智能指针别名 `S<T>=shared_ptr`、`U<T>=unique_ptr`、`W<T>=weak_ptr`。
