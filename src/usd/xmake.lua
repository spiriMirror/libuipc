-- Keep the package above the security floor recorded in agent_docs/09.
add_requires("usd >=25.08")

target("uipc_usd")
    add_rules("component")
    add_files("**.cpp")
    add_headerfiles(path.join(os.projectdir(), "include/uipc/usd/**.h"))
    add_includedirs(os.scriptdir())
    add_deps("uipc_core", "uipc_geometry")
    add_packages("usd")
