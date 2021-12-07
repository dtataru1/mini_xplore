from shapely.geometry import Polygon
from shapely.ops import unary_union
import pyvisgraph as vg
import numpy as np

#polygon1 = Polygon([(0,0), (1,1), (1,0)])
#print(np.size(polygon1.exterior.coords))
#polygon2 = Polygon([(0,1), (1,0), (1,1)])
#polygons = [polygon1, polygon2]
#polys = [vg.Point(0,0), vg.Point(1,1), vg.Point(1,0)]
#print(polys)

def unify(p1,p2):
    polygon1 = vg2shapely(p1)
    polygon2 = vg2shapely(p2)
    polygons = [polygon1, polygon2]
    u = unary_union(polygons)
    return shapely2vg(u)

def check_overlap(p1,p2):
    polygon1 = vg2shapely(p1)
    polygon2 = vg2shapely(p2)
    return polygon1.intersects(polygon2)

def vg2shapely(vg_polygon):
    vect = []
    for p in vg_polygon:
        vect.append((p.x,p.y))
    return Polygon(vect)

def shapely2vg(shapely_polygon):
    vg_polygon = []
    for p in range(0,int((np.size(shapely_polygon.exterior.coords)-2)/2)):
        x,y = shapely_polygon.exterior.coords[p]
        vg_polygon.append(vg.Point(x,y))
    return vg_polygon
