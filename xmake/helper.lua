-- @param prefix (string) "cp", "cu"
-- @param version_str (string) "3.10" "12.4"
-- @return (string) "cp310", "cu124"
function format_version_with_prefix(prefix, version_str)
    if type(prefix) ~= "string" or type(version_str) ~= "string" then
        return nil, "Invalid input: prefix and version_str must be strings."
    end

    local number_part = string.gsub(version_str, "%.", "")
    
    return prefix .. number_part
end

function get_cuda_version()
    import("core.base.option")
    import("core.project.config")
    import("lib.detect.find_programver")
    import("detect.sdks.find_cuda")

    local cuda_version = option.get("cuda_version")
    if not cuda_version then
        local cuda = assert(find_cuda(config.get("cuda")))
        cuda_version = find_programver(path.join(cuda.bindir, "nvcc"), {parse = "release (%d+%.%d+),"})
    end
    return cuda_version
end

function get_python_version(target)
    import("lib.detect.find_tool")
    import("lib.detect.find_programver")

    local envs = target:pkgenvs()
    local python = assert(find_tool("python3", {envs = envs}), "python not found!")
    return find_programver(python.program)
end
