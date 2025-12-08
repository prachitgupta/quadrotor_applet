
# debug_pluto.jl
using Pkg
Pkg.activate(".")
using Pluto
println(names(Pluto))
# Also try a broader search for "build" or "html" related functions
for n in names(Pluto, all=true)
    if occursin("build", string(n)) || occursin("html", string(n)) || occursin("export", string(n)) || occursin("static", string(n))
        println(n)
    end
end
