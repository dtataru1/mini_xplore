### SOURCES:
# tqdm (https://pypi.org/project/tqdm/): pip install tqdm
# pyvisgraph (https://github.com/TaipanRex/pyvisgraph): pip install pyvisgraph
# shapely (https://pypi.org/project/Shapely/): REQUIRES GEOS: brew install geos pip install shapely

import numpy as np
import matplotlib.pyplot as plt
import pyvisgraph as vg
from pyvisgraph import visible_vertices as vv
import merge_obstacles as mo
import cv2
from pyvisgraph.graph import Point
import math
from constants import *
import pyclipper


def increase_coordinates(polygon):
    subj = []
    for p in polygon:
        subj.append((math.floor(p.x),math.floor(p.y)))
    subj = tuple(subj)
    pco = pyclipper.PyclipperOffset()
    pco.AddPath(subj, pyclipper.JT_MITER, pyclipper.ET_CLOSEDPOLYGON)
    solution = pco.Execute(OBST_INCREASE)
    polygon = []
    for i in range(len(solution[0])):
        polygon.append(Point(solution[0][i][0],solution[0][i][1]))
    return polygon

def draw_polygon(image, polygon, color, size, complete=True):
    local_polygon = [i for i in polygon]
    if complete:
        local_polygon.append(local_polygon[0])
    p1 = local_polygon[0]
    for p2 in local_polygon[1:]:
        cv2.line(image, (int(p1.x), int(p1.y)), (int(p2.x), int(p2.y)), color=color, thickness=size)
        p1 = p2

def draw_visible_vertices(image, edges, color, size):
    for e in edges:
        cv2.line(image, (int(e.p1.x), int(e.p1.y)), (int(e.p2.x), int(e.p2.y)), color=color, thickness=size)

def check_point_obstacles_overlap(point,graph,howmuch):
    intersected_polygon = vv.point_in_polygon(point,graph)
    if intersected_polygon != -1:
        return vv.closest_point(point, graph, intersected_polygon, length=howmuch)
    else: return point

def draw_field_borders(image, polys):
    polys.append([vg.Point(0,100), vg.Point(0,690)])
    polys.append([vg.Point(100,0), vg.Point(1330,0)])
    polys.append([vg.Point(1430,100), vg.Point(1430,680)])
    polys.append([vg.Point(100,790), vg.Point(1330,790)])

    for p in polys[len(polys)-4:]:
        draw_polygon(image, p, (0,100,100), 6, complete=False)

# Global Planner based on visibility graph (vg)
def global_path(image, polys, thymio_position, end_position, compute_global, path):

    g = vg.VisGraph()
    g.build(polys)

    if compute_global:
        # Increasing obstacle borders
        if polys != []:
            print('polys before', polys)
            for i in range(len(polys)):
                polys[i]=increase_coordinates(polys[i])

            print('polys', polys)
            # Checking obstacles overlaping to merge them
            i = 0
            while i < len(polys):
                j = 0
                while j < len(polys):
                    if j != i:
                        if mo.check_overlap(polys[j],polys[i]):
                            polys[j] = mo.unify(polys[j],polys[i])
                            polys.pop(i)
                            i = 0
                            j = len(polys)
                        else:
                            j += 1
                    else: j += 1
                i +=1

    # Drawing the new obstacles (possibly merged)
    if len(polys) > 0:
        for polygon in polys:
            draw_polygon(image, polygon,(0,255,0),6)

        # Updating polygons map
        g.build(polys)

        # Showing the visibility graph
        draw_visible_vertices(image, g.visgraph.get_edges(),(50,50,50), 2)

    if compute_global:
        # Computing the shortest path without the starting point (current thymio position)
        path = g.shortest_path(thymio_position, end_position)
        path.pop(0)

    return path
