using Plots

vx = 22.2
vy = 22.2
x0 = -80
y0 = -80
l0 = sqrt(x0^2 + y0^2)
θ = atan(x0/y0)

FPS = 30 
dt = 1/FPS
dx = vx*dt
dy = vy*dt
x1 = x0 + dx
y1 = y0 + dy
l1 = sqrt(x1^2+y1^2)

v = sqrt(vx^2+vy^2)
g = vx*x0+vy*y0
dl = l1 - l0

npixels_x = 2*x0 / dx
npixels_y = 2*y0*tan(π/12) / dy

al_stop_f = 10 # Required number of frames for algorithm determine to stop of not.

stop_t = al_stop_f / FPS

d_lidar = (vy*stop_t + abs(y0)) * csc(θ)
dot_density = 6.34 # unit = 1/m^2.

angular_resolution = atan(1/d_lidar/sqrt(dot_density)) * 180 / π

println("x = $x0")
println("y = $y0")
println("l = $l0")
println("dt = $dt s")
println("FPS = $(1/dt)")
println("dx: $dx m")
println("dy: $dy m")
println("dl: $dl m")
println("d_lidar: $d_lidar")
println("angular_resolution: $angular_resolution")
#println("Required least number of pixels along x: $npixels_x")
#println("Required least number of pixels along y: $npixels_y")

vy = range(0,100, length=101)
fps = 10
d_lidar = (vy.*(1/fps) .+ abs(y0)) .* csc(θ)

p = plot(vy, d_lidar, label="$fps FPS")
for fps in [20, 25, 30]
    d_lidar = ((vy .* 1000 ./ 3600).*(1/fps) .+ abs(y0)) .* csc(θ)
    plot!(vy, d_lidar, label="$fps")
end
ylabel!("Distance (m)")
xlabel!("Velocity (km/h)")
savefig("test.png")