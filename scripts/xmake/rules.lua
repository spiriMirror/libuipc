rule("pybind")
on_load(function (target)
    target:set("kind", "shared")
    target:set("extension", ".pyd")
    cfg_path = path.join(os.scriptdir(), "pythoninclude.txt")
    cfg = io.open(cfg_path)
    local py_include = cfg:read()
    cfg_path = path.join(os.scriptdir(), "pythonlink.txt")
    cfg = io.open(cfg_path)
    local py_linkdir = cfg:read()
    local py_libs = nil
    if not py_include then
        print("Python include path not found")
        os.exit(1)
    end
    local files = {}
    for _, filepath in ipairs(os.files(path.join(py_linkdir, "*.lib"))) do
        local lib_name = path.basename(filepath)
        table.insert(files, lib_name)
    end
    py_libs = files
    target:add("includedirs", py_include)
    target:add("linkdirs", py_linkdir)
    target:add("syslinks", py_libs)
    -- start RTTI
    target:add("cxxflags", "/GR")
end)
after_build(function(target)
    -- copy the MSVCP140.dll and VCRUNTIME140.dll to the output directory
end)
rule_end()