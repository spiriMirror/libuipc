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