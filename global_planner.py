import numpy as np
import matplotlib.pyplot as plt
import pyvisgraph as vg
from pyvisgraph import visible_vertices as vv
import merge_obstacles as mo
import cv2
from pyvisgraph.graph import Point
import math
from constants import *

def increase_coordinates(polygon, howmuch):
    center_x = 0
    center_y = 0
    for p in polygon:
        center_x += p.x/np.size(polygon)
        center_y += p.y/np.size(polygon)
    for p in polygon:
        angle = math.atan2(p.y-center_y,p.x-center_x)
        #p.x = p.x + np.sign(math.cos(angle))*howmuch
        #p.y = p.y + np.sign(math.sin(angle))*howmuch
        p.x = math.floor(p.x + math.cos(angle)*howmuch)
        p.y = math.floor(p.y + math.sin(angle)*howmuch)

def draw_polygon(image, polygon, color, size, complete=True):
    local_polygon = [i for i in polygon]
    #print('PATH', polygon)
    if complete:
        local_polygon.append(local_polygon[0])
    p1 = local_polygon[0]
    for p2 in local_polygon[1:]:
        #plt.plot([p1.x, p2.x], [p1.y, p2.y], color=color, linestyle='solid',  linewidth=size)
        cv2.line(image, (int(p1.x), int(p1.y)), (int(p2.x), int(p2.y)), color=color, thickness=size)
        p1 = p2

def draw_visible_vertices(image, edges, color, size):
    for e in edges:
        cv2.line(image, (int(e.p1.x), int(e.p1.y)), (int(e.p2.x), int(e.p2.y)), color=color, thickness=size)
        #plt.plot([e.p1.x, e.p2.x], [e.p1.y, e.p2.y], color=color, linestyle='dashed',  linewidth=size)

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

def global_path(image, polys, thymio_position, end_position, compute_global, path):

    g = vg.VisGraph()
    g.build(polys)

    # Increasing distance from obstacles for starting and ending points
    #start_point = check_point_obstacles_overlap(thymio_position,g.graph,65)
    #end_point = check_point_obstacles_overlap(end_position,g.graph,65)

    if compute_global:
        # Increasing obstacle borders
        if polys != []:
            for p in polys:
                increase_coordinates(p, 100)

            # Checking obstacles overlaping to merge them
            i = 0
            while i < len(polys):
                j = 0
                while j < len(polys):
                    if j != i:
                        if mo.check_overlap(polys[j],polys[i]):
                            # polys[j]: [Point(749.00, 367.00), Point(580.00, 455.00), Point(600.00, 455.00)]
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
            draw_polygon(image, polygon,(0,100,100),6)

        #draw_field_borders(image, polys)
        # Updating polygons map
        g.build(polys)

        # Showing the visibility graph
        draw_visible_vertices(image, g.visgraph.get_edges(),(50,50,50), 2)

    if compute_global:
        # Computing the shortest path
        path = g.shortest_path(thymio_position, end_position)
        path.pop(0)

        # Exporting the path sequence (without starting point)

    return path
