import sys

from grid import *
from particle import Particle
from utils import *
from setting import *
import math
import numpy as np
import random

# ------------------------------------------------------------------------
def motion_update(particles, odom):
    """ Particle filter motion update

        Arguments: 
        particles -- input list of particle represents belief p(x_{t-1} | u_{t-1})
                before motion update
        odom -- noisy odometry measurement, a pair of robot pose, i.e. last time
                step pose and current time step pose

        Returns: the list of particle represents belief \tilde{p}(x_{t} | u_{t})
                after motion update
    """

    # Old robot odometry with added noise [x, y, h]
    old_robo_x = odom[0][0]
    old_robo_y = odom[0][1]
    old_robo_h = odom[0][2]
    
    # New robot odometry with added noise [x, y, h]
    new_robo_x = odom[1][0]
    new_robo_y = odom[1][1]
    new_robo_h = odom[1][2]

    rot1_pred = diff_heading_deg(math.atan2(new_robo_y - old_robo_y, new_robo_x - old_robo_x),
                                 old_robo_h)
    trans_pred = math.sqrt(((old_robo_x - new_robo_x)**2) + ((old_robo_y - new_robo_y)**2))
    rot2_pred = diff_heading_deg(new_robo_h - old_robo_h, rot1_pred)

    if odom[0] != odom[1]:
        for particle in particles:
            alpha1 = 0.01
            alpha2 = 0.01
            alpha3 = 0.05
            alpha4 = 0.05

            rot1_rand = add_gaussian_noise(rot1_pred - ((rot1_pred * alpha1) + (trans_pred * alpha2)), ODOM_HEAD_SIGMA)
            trans_rand = add_gaussian_noise(trans_pred - ((trans_pred * alpha3) + (alpha4 * (rot1_pred + rot2_pred))),
                                                         ODOM_TRANS_SIGMA)
            rot2_rand = add_gaussian_noise(rot2_pred - ((rot2_pred * alpha1) + (trans_pred * alpha2)), ODOM_HEAD_SIGMA)
            particle.move(rot1_rand, trans_rand, rot2_rand)

    return particles

# ------------------------------------------------------------------------
def measurement_update(particles, measured_marker_list, grid):
    """ Particle filter measurement update

        Arguments: 
        particles -- input list of particle represents belief \tilde{p}(x_{t} | u_{t})
                before meansurement update
        measured_marker_list -- robot detected marker list, each marker has format:
                measured_marker_list[i] = (rx, ry, rh)
                rx -- marker's relative X coordinate in robot's frame
                ry -- marker's relative Y coordinate in robot's frame
                rh -- marker's relative heading in robot's frame, in degree
        grid -- grid world map, which contains the marker information, 
                see grid.h and CozGrid for definition

        Returns: the list of particle represents belief p(x_{t} | u_{t})
                after measurement update
    """
    particle_weights = [1 for _ in range(len(particles))]
    for i in range(len(particles)):
        particle = particles[i]
        markers_visible_to_particle = particle.read_markers(grid)
        marker_pairs = []
        already_paired_markers = []
        for cm in measured_marker_list:
            closest_marker = None
            closest_distance = sys.maxsize
            for pm in markers_visible_to_particle:
                if grid_distance(cm[0], cm[1], pm[0], pm[1]) < closest_distance and pm not in already_paired_markers:
                    closest_marker = pm
                    closest_distance = grid_distance(cm[0], cm[1], pm[0], pm[1])
            if closest_marker is not None:
                marker_pairs.append([cm, closest_marker])
                already_paired_markers.append(closest_marker)

        if len(marker_pairs) != 0:
            particle_prob = get_particle_prob(marker_pairs)
        elif len(markers_visible_to_particle) != len(measured_marker_list):
            particle_prob = 0
        elif not grid.is_in(particle.x, particle.y):
            particle_prob = 0
        else:
            #both lists should have len 0
            particle_prob = 1

        particle_weights[i] = particle_prob

    ## Normalization
    prob_sum = 0.0
    for weight in particle_weights:
        prob_sum += weight
    if prob_sum != 0: 
        for i in range(len(particle_weights)):
            particle_weights[i] = particle_weights[i] / prob_sum

        ## Resampling
        measured_particles = []

        for i in range(0, PARTICLE_COUNT):
            rand = np.random.choice(particles, p=particle_weights)
            x,y,h = rand.xyh
            new_particle = Particle(add_gaussian_noise(x,MARKER_TRANS_SIGMA),add_gaussian_noise(y,MARKER_TRANS_SIGMA),add_gaussian_noise(h,MARKER_ROT_SIGMA))
            measured_particles.append(new_particle)
    else:
        #Something went wrong! need to start over with random sample.
        return Particle.create_random(PARTICLE_COUNT,grid)

    return measured_particles

def get_particle_prob(marker_pairs):
    """
    Expected input: 
    [ (measured marker, actual marker), (), (), ... ]
    all markers should be in (x,y,h) format
    """
    prob = 1.0

    for pair in marker_pairs:
        dist = grid_distance(pair[0][0], pair[0][1], pair[1][0], pair[0][1])
        diff_heading = diff_heading_deg(pair[0][2], pair[1][2])
        exp = -1 * ( ((dist**2)/((2*MARKER_TRANS_SIGMA)**2)) + ((diff_heading**2)/((2*MARKER_ROT_SIGMA)**2)) )
        prob = prob * math.exp(exp)
    return prob