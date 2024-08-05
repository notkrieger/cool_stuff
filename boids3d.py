import pygame
import numpy as np

N = 100 # number of boids
total_time = 1
time_steps = 10000
dt = total_time/time_steps
width = 720
height = 480
depth = 100
speed_max = 1
local_r = 40 # distance for other boids to be in locality
boids = []
sep_factor = 0.025
cohesion_factor = 0.01
align_factor = 0.1
trail_len = 10
trails = np.zeros((trail_len, N, 3)) # trails - last X positions of boids


class Boid:
    def __init__(self):
        self.x = np.random.random() * width
        self.y = np.random.random() * height
        self.z = np.random.random() * depth
        self.vx = np.random.normal(0, 5) * speed_max
        self.vy = np.random.normal(0, 5) * speed_max
        self.vz = np.random.normal(0, 5) * speed_max


def distance(boid, other):
    return ((boid.x - other.x) ** 2 + (boid.y - other.y) ** 2 + (boid.z - other.z) ** 2) ** 0.5


def draw_boids(boids, screen, t):
    screen.fill((0, 0, 0))
    red = (255, 0, 0)
    radius = 5
    red_fade = (255, 160, 153)

    i = 0  # trail factor for drawing
    for timestep in trails:
        j = 0
        for boid in boids:
            order = (t - i) % trail_len
            pygame.draw.circle(screen, red_fade, (trails[i][j][0], trails[i][j][1]),
                               radius * (trail_len - order) / trail_len * (depth - boid.z)/depth + 1)
            j += 1
        i += 1

    for boid in boids:
        pygame.draw.circle(screen, red, (boid.x, boid.y), radius * (depth - boid.z)/depth + 1.5)
        # draw further away circles as smaller


def sep(boid):
    # avoid other boids
    sep_dir = [0, 0, 0]

    for other in boids:
        r = distance(boid, other)
        if r > local_r or other == boid:
            continue
        sep_dir[0] += (local_r / (boid.x - other.x)) # should hopefully make it so close --> more "push"
        sep_dir[1] += (local_r / (boid.y - other.y))
        sep_dir[2] += (local_r / (boid.z - other.z))

    boid.vx += sep_dir[0] * sep_factor
    boid.vy += sep_dir[1] * sep_factor
    boid.vz += sep_dir[2] * sep_factor


def cohesion(boid):
    # go towards centre of pack
    centre = [0, 0, 0]
    num_neighbours = 0
    for other in boids:
        r = distance(boid, other)
        if r < local_r:
            centre[0] += other.x
            centre[1] += other.y
            centre[2] += other.z
            num_neighbours += 1


    if num_neighbours > 0:
        centre[0] /= num_neighbours
        centre[1] /= num_neighbours
        centre[2] /= num_neighbours
        boid.vx += (centre[0] - boid.x) * cohesion_factor
        boid.vy += (centre[1] - boid.y) * cohesion_factor
        boid.vz += (centre[2] - boid.z) * cohesion_factor


def align(boid):
    ave_align = [0, 0, 0]
    num_neighbours = 0
    for other in boids:
        r = distance(boid, other)
        if r > local_r:
            continue
        num_neighbours += 1
        ave_align[0] += other.vx
        ave_align[1] += other.vy # calculate average alignment
        ave_align[2] += other.vz

    if num_neighbours > 0:
        ave_align[0] /= num_neighbours
        ave_align[1] /= num_neighbours
        ave_align[2] /= num_neighbours
        boid.vx += (ave_align[0] - boid.vx) * align_factor
        boid.vy += (ave_align[1] - boid.vy) * align_factor
        boid.vz += (ave_align[2] - boid.vz) * align_factor


def limit_speed(boid):
    speed = (boid.vx ** 2 + boid.vy ** 2 + boid.vz ** 2) ** 0.5
    if speed > speed_max:
        boid.vy /= speed * speed_max
        boid.vx /= speed * speed_max
        boid.vz /= speed * speed_max


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
    if boid.z < local_r:
        boid.vz += turn
    if boid.z > depth + local_r:
        boid.vz -= turn


def main():
    for i in range(N):
        # maybe some meshgrid to make N x N grid??
        boids.append(Boid())

    screen = pygame.display.set_mode((width, height))

    draw_boids(boids, screen, 0)

    for t in range(time_steps):
        j = 0
        for boid in boids:
            # separation - steer to avoid crowding local flockmates
            # alignment - steer towards average heading of flockmates
            # cohesion - steer towards centre of mass of local flockmates
            align(boid)
            cohesion(boid)
            sep(boid)
            limit_speed(boid)
            # redirect boids away from edge of screen
            redirect(boid)
            # works quickly and is easy but does not look great
            # would need a queue to make look pretty
            trails[t % trail_len][j][0] = boid.x
            trails[t % trail_len][j][1] = boid.y
            trails[t % trail_len][j][2] = boid.z
            # update position
            boid.x += boid.vx
            boid.y += boid.vy
            boid.z += boid.vz

            j += 1

        draw_boids(boids, screen, t) # redraw the boids
        # write the time step on the screen somewhere
        pygame.display.flip()


main()

