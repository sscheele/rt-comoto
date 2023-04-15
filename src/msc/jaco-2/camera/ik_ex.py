#! /usr/bin/env python3

from ikpy.chain import Chain
import numpy as np
import ikpy.utils.plot as plotter

jaco = Chain.from_urdf_file("jaco.urdf", base_elements=['j2s7s300_link_base'])
target_pos = [0.5, 0.5, 0.1]
print(jaco.inverse_kinematics(target_pos))
