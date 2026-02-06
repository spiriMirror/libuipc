--[[
        A (target/binary) -> uipc_geometry(target/shared) -> tbb (package/shared)
        The GNU linker (ld) requires a complete dependency chain, but xmake doesn't pass the -lC flag for target A during the linking stage
--]]
if not is_plat("windows") then
    add_packages("tbb")
end
includes("app")
if has_config("examples") then 
    includes("examples")
end
if has_config("tests") then
    includes("tests")
end
if has_config("benchmarks") then
    includes("benchmarks")
end

includes('rbc')