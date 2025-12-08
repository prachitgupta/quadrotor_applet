
# build_pluto.jl
using Pluto

notebook_path = "/home/prachit/quadrotor_applet/quadrotor_simulation.jl"
# Output file name will be the same as the notebook, but with .html extension
output_html_filename = replace(basename(notebook_path), ".jl" => ".html")
build_dir_path = "/home/prachit/quadrotor_applet/docs"
output_html_path = joinpath(build_dir_path, output_html_filename)

# Create the docs directory if it doesn't exist
mkpath(build_dir_path)

# Using export_notebook with keyword arguments
Pluto.export_notebook(
    notebook_path,
    output_path=output_html_path
)

println("Pluto notebook built successfully to $output_html_path")
