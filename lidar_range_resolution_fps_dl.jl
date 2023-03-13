vx = 22.2
vy = 22.2
# x = 100*sqrt(3)
# y = 100
x = 80*√3
y = 80
dl = -1.4
l = sqrt(x^2 + y^2)
v = sqrt(vx^2+vy^2)
g = vx*x+vy*y
dt = 1/v^2 * (g - sqrt(g^2 + v^2*(2*dl*l+dl^2)))

println("x = $x")
println("y = $y")
println("l = $l")
println("dt = $dt s")
println("FPS = $(1/dt)")

resolution_x = vx * dt
resolution_y = vy * dt

println("x resolution: $resolution_x m")
println("y resolution: $resolution_y m")

npixels_x = 2*x / resolution_x
npixels_y = 2*y*tan(π/12) / resolution_y

println("Required least number of pixels along x: $npixels_x")
println("Required least number of pixels along y: $npixels_y")