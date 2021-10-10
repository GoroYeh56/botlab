"""
Particle filters are not easy to tune but one very helpful indicator
of how to tweak your model's parameters is by visualizing the progression
of particles. 
This tool helps one do so by visualizing x, y, and theta seperatel.

To run the code
    $ python particle_plotter.py <log_file_location.log>

Potential code expantions:
    - Allow the code to monitor live channels given certain command line arguments.
    - plot other useful information like forward and angular speed.
"""

import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
sys.path.append("lcmtypes")
import lcm
from lcmtypes import particles_t
from lcmtypes import pose_xyt_t
from time import sleep
from copy import deepcopy


def get_axis_settings(particles, ground_truth, min_axis_width=1, bin_width=0.05, necessary_padding=0.05):
    min_x = min(min(particles), ground_truth)
    max_x = max(max(particles), ground_truth)
    bins_width = max_x - min_x
    axis_padding = max((min_axis_width - bins_width) / 2, necessary_padding)
    min_x = min_x - axis_padding
    max_x = max_x + axis_padding
    n_bins = round((bins_width) / bin_width)
    return [min_x, max_x], n_bins

if __name__ == '__main__':

    if len(sys.argv) < 2:
        sys.stderr.write("usage: mbot_controller_plot.py <logfile>")
        sys.exit(1)
    log = lcm.EventLog(sys.argv[1],"r")

    particles_dict_default = {'weights':[], 'x':[], 'y':[], 'theta':[]}
    show_axis, show_left_most_axis = False, True

    for event in log:
        channels_of_particles_dict = {}

        if event.channel == "TRUE_POSE":
            msg = pose_xyt_t.decode(event.data)
            truth_x = (msg.x)
            truth_y = (msg.y)
            truth_theta = (msg.theta)
        
        if event.channel.find('PARTICLES') != -1:
            msg = particles_t.decode(event.data)
            channels_of_particles_dict[event.channel] = deepcopy(particles_dict_default)
            for i in range(msg.num_particles):
                channels_of_particles_dict[event.channel]['weights'].append(msg.particles[i].weight)
                channels_of_particles_dict[event.channel]['x'].append(msg.particles[i].pose.x)
                channels_of_particles_dict[event.channel]['y'].append(msg.particles[i].pose.y)
                channels_of_particles_dict[event.channel]['theta'].append(msg.particles[i].pose.theta)

            # value below has 3 digits first indicating columns and second rows that will be needed.
            subplot_matrix_index = len(channels_of_particles_dict)*100 + 31

            for channel_name, particles in channels_of_particles_dict.items():
                
                limits, n_bins = get_axis_settings(particles['x'], truth_x)
                ax = plt.subplot(subplot_matrix_index)
                plt.ylabel(channel_name)
                plt.hist(particles['x'], weights=particles['weights'], label='slam', color='b', bins=n_bins)
                plt.xlabel("X")
                plt.xlim(limits)
                plt.axvline(truth_x, color='r', linestyle='dashed', linewidth=1)
                ax.get_yaxis().set_visible(show_axis or show_left_most_axis)

                limits, n_bins = get_axis_settings(particles['y'], truth_y)
                ax = plt.subplot(subplot_matrix_index + 1)
                plt.hist(particles['y'], weights=particles['weights'],label='slam', color='b', bins=n_bins)
                plt.xlabel("Y")
                plt.xlim(limits)
                plt.axvline(truth_y, color='r', linestyle='dashed', linewidth=1)
                ax.get_yaxis().set_visible(show_axis)
                
                limits, n_bins = get_axis_settings(particles['theta'], truth_theta)
                ax = plt.subplot(subplot_matrix_index + 2)
                plt.hist(particles['theta'], weights=particles['weights'],label='slam', color='b', bins=n_bins)
                plt.xlabel("Theta")
                plt.xlim(limits)
                plt.axvline(truth_theta, color='r', linestyle='dashed', linewidth=1)
                ax.get_yaxis().set_visible(show_axis)
                
                subplot_matrix_index = subplot_matrix_index + 3

            plt.draw()
            plt.pause(0.000005)
            plt.clf()
