import pygame
import numpy as np

N = 100 # number of boids
total_time = 1
time_steps = 10000
dt = total_time/time_steps
width = 720
height = 480
speed_max = 1
local_r = 40 # distance for other boids to be in locality
boids = []
trail_len = 15
trails = np.zeros((trail_len, N, 2)) # trails - last X positions of boids
sep_factor = 0.025
cohesion_factor = 0.01
align_factor = 0.1
wind_factor = 0.05
wind_period = 7
wind_sin = 700


class Boid:
    def __init__(self):
        self.x = np.random.random() * width
        self.y = np.random.random() * height
        self.vx = np.random.normal(0, 5) * speed_max
        self.vy = np.random.normal(0, 5) * speed_max


def distance(boid, other):
    return ((boid.x - other.x) ** 2 + (boid.y - other.y) ** 2) ** 0.5


def draw_boids(boids, trails, screen, t):
    screen.fill((0, 0, 0))
    red = (255, 0, 0)
    radius = 4
    red_fade = (255, 160, 153)

    # draw trail
    i = 0 # trail factor for drawing
    for timestep in trails:
        j = 0
        for boid in boids:
            order = (t - i) % trail_len
            pygame.draw.circle(screen, red_fade, trails[i][j],
                               radius * (trail_len - order)/trail_len)
            j += 1
        i -= 1

    for boid in boids:
        pygame.draw.circle(screen, red, (boid.x, boid.y), radius)


def update_v(boid):

    sep_dir = [0, 0] # avoid other boids
    ave_align = [0, 0] # align with nearby boids
    centre = [0, 0] # go toward centre of pack
    num_neighbours = 0

    for other in boids:
        r = distance(boid, other)
        if r > local_r or other == boid:
            continue
        num_neighbours += 1
        sep_dir[0] += (local_r / (boid.x - other.x))  # should hopefully make it so close --> more "push"
        sep_dir[1] += (local_r / (boid.y - other.y))
        centre[0] += other.x
        centre[1] += other.y
        ave_align[0] += other.vx
        ave_align[1] += other.vy  # calculate average alignment

    # separate
    boid.vx += sep_dir[0] * sep_factor
    boid.vy += sep_dir[1] * sep_factor
    if num_neighbours > 0:
        # centre
        centre[0] /= num_neighbours
        centre[1] /= num_neighbours
        boid.vx += (centre[0] - boid.x) * cohesion_factor
        boid.vy += (centre[1] - boid.y) * cohesion_factor
        # align
        ave_align[0] /= num_neighbours
        ave_align[1] /= num_neighbours
        boid.vx += (ave_align[0] - boid.vx) * align_factor
        boid.vy += (ave_align[1] - boid.vy) * align_factor


def limit_speed(boid):
    speed = (boid.vx ** 2 + boid.vy ** 2) ** 0.5
    if speed > speed_max:
        boid.vy /= speed * speed_max
        boid.vx /= speed * speed_max


def redirect(boid): # make boid avoid htting edges of screen
    turn = 0.5

    if boid.x < local_r:
        boid.vx += turn
    if boid.x > width - local_r:
        boid.vx -= turn
    if boid.y < local_r:
        boid.vy += turn
    if boid.y > height - local_r:
        boid.vy -= turn


def wind_uv(x, y, t):
    u = (y + height / 2) / height * np.sin(y * t / wind_sin) * wind_factor
    v = (x + width / 2) / width * np.cos(x * t / wind_sin) * wind_factor
    return u, v


def wind(boid, t):
    t1 = t // wind_period
    u, v = wind_uv(boid.x, boid.y, t1)
    boid.vx += u
    boid.vy += v


def draw_wind(screen, t):
    t1 = t // wind_period # to make wind visible on the screen
    white = (255, 255, 255)
    x_points = np.linspace(1, width, 30)
    y_points = np.linspace(1, height, 20)
    for x in x_points:
        for y in y_points:
            u, v = wind_uv(x, y, t1)
            norm = (u**2 + v**2)**0.5 / 10
            pygame.draw.line(screen, white, (x, y), (x+u/norm, y+v/norm))
            pygame.draw.circle(screen, white, (x, y), 2)


def main():
    for i in range(N):
        # maybe some meshgrid to make N x N grid??
        boids.append(Boid())

    screen = pygame.display.set_mode((width, height))

    draw_boids(boids, trails, screen, 0)

    for t in range(time_steps):
        j = 0
        for boid in boids:
            # separation - steer to avoid crowding local flockmates
            # alignment - steer towards average heading of flockmates
            # cohesion - steer towards centre of mass of local flockmates
            update_v(boid)
            limit_speed(boid)
            wind(boid, t)
            # redirect boids away from edge of screen
            redirect(boid)
            trails[int(t % trail_len)][j][0] = boid.x
            trails[int(t % trail_len)][j][1] = boid.y
            # update position
            boid.x += boid.vx
            boid.y += boid.vy

            j += 1

        draw_boids(boids, trails, screen, t) # redraw the boids
        draw_wind(screen, t)
        # write the time step on the screen somewhere
        pygame.display.flip()


main()

