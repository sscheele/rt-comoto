#!/usr/bin/env python3
"""
Removes the 'visual' and 'collision' tags from an
urdf file so it can be used for fk with no collision
checking or visualization  
"""

import sys

if __name__ == "__main__":
    banned_tags = ['visual', 'collision']
    with open(sys.argv[1], 'r') as urdf:
        with open('novis-' + sys.argv[1], 'w') as out:
            in_vis = False
            for line in urdf:
                for tag in banned_tags:
                    line = line.replace(f"<{tag}>", f"<!-- <{tag}>")
                    line = line.replace(f"</{tag}>", f"</{tag}> -->")
                print(line, file=out, end='')
        
