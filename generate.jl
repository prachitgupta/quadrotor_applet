using Pluto, PlutoStaticHTML

const NOTEBOOK_PATH = "quadrotor_simulation.jl"
const OUTPUT_DIR = "docs"

function build()
    println("Building notebook...")
    PlutoStaticHTML.build_notebooks(
        PlutoStaticHTML.BuildOptions(
            ".",
            [NOTEBOOK_PATH],
            OUTPUT_DIR
        )
    )
    println("Build complete!")
end

build()
