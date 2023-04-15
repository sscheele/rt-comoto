#!/usr/bin/env python3
import urdfpy

EE_LINK_NAME = "j2s7s300_ee_base"
    
def get_jaco_fk(urdf, x):
    """
    Reads current jaco joint state and gets the link 7 fk
    """
    link_map = {f'Actuator{i}': x[i-1] for i in range(1,8)}
    fk_map = urdf.link_fk(cfg=link_map)
    for link, matr in fk_map.items():
        if link.name == EE_LINK_NAME:
            print(matr)
            return matr[:-1,3]
    raise KeyError("No link named " + EE_LINK_NAME)
        

if __name__ == "__main__":
    urdf = urdfpy.URDF.load('novis-jaco.urdf')
    
    point = [-1.3990884828798196, 0.11365406136563036, -0.22443097664759293, -2.5635543379364214, -0.1567893481206708, -1.0254485798383568, 1.5886885082624849]
    print(get_jaco_fk(urdf, point))
