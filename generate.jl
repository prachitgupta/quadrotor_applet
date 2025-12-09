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
    # Rename the output file to index.html for GitHub Pages
    mv(joinpath(OUTPUT_DIR, "quadrotor_simulation.html"), joinpath(OUTPUT_DIR, "index.html"))
    println("Build complete!")
end

build()
