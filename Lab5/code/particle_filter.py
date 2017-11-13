from .grid import *
from .particle import Particle
from .utils import *
from .setting import *

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
    rot2_pred = diff_heading_deg(diff_heading_deg(new_robo_h, old_robo_h), rot1_pred)
    particle_mean = compute_mean_pose(particles)

    if odom[0] != odom[1]:
        for particle in particles:
            alpha1 = 1.0
            alpha2 = 1.0
            alpha3 = 1.0
            alpha4 = 1.0



            x, y, h = particle.xyh





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
    measured_particles = []
    return measured_particles
