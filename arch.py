#!/usr/bin/env python
# -*- coding: utf-8 -*-

from compas.geometry import Point, Line
from compas.datastructures import Mesh

from compas_assembly.datastructures import Block
from compas_assembly.datastructures import Assembly
from compas_assembly.datastructures import assembly_interfaces_numpy

from compas_assembly.geometry import Arch
from compas.geometry import Box
from compas.geometry import Frame

from compas_cra.datastructures import CRA_Assembly

from compas_view2.app import App
from compas_view2.objects import Object
from compas_view2.objects import MeshObject

from compas.geometry import Rotation
from compas.geometry import add_vectors
from compas.geometry import subtract_vectors
from compas.geometry import transform_points
from compas.geometry import angle_vectors
from math import radians

def middle_point(list1, list2):
    list3 = []
    for i,_ in enumerate(list1):
        b = (list1[i] + list2[i])/2 
        list3.append(b)
    return list3
# construct an arch assembly

class CRA_Arch(Arch):
    ## Extend the data structure of Arch for CRA Analysis

    ##Create interface and related keys
    def interface(self):  
        interfaces = []
        keys = []
        blocks = self.blocks()
        for i, block in enumerate(blocks):
            if i!=0:
                interface = Mesh()
                for j,v in enumerate(block.face_vertices(0)):
                    coor = block.vertex_coordinates(v)
                    interface.add_vertex(key=j, x=coor[0], y=coor[1], z=coor[2])
                
                interface.add_face([k for k,_ in enumerate(block.face_vertices(0))])

                interfaces.append(interface)
                keys.append((i-1,i))
        
        return interfaces, keys
    
        ##Return the bounding box of the arch
    def bounding_box(self):

        return Box(Frame([0,self.depth/2,(self.rise+self.thickness)/2], [1,0,0],[0,1,0]), 
                    xsize = self.span + self.thickness*2,
                    ysize = self.depth,
                    zsize = self.rise + self.thickness
                     )
        ##Adding wedges to the geometry

    def wedged_block(self, num, left_angle = 0, right_angle = 0):
        
        radius = self.rise / 2 + self.span**2 / (8 * self.rise)
        # base = [0.0, 0.0, 0.0]
        top = [0.0, 0.0, self.rise]
        left = [- self.span / 2, 0.0, 0.0]
        center = [0.0, 0.0, self.rise - radius]
        vector = subtract_vectors(left, center)
        springing = angle_vectors(vector, [-1.0, 0.0, 0.0])
        sector = radians(180) - 2 * springing
        angle = sector / self.n

        a = top
        b = add_vectors(top, [0, self.depth, 0])
        c = add_vectors(top, [0, self.depth, self.thickness])
        d = add_vectors(top, [0, 0, self.thickness])

        R = Rotation.from_axis_and_angle([0, 1.0, 0], 0.5 * sector, center)
        bottom = transform_points([a, b, c, d], R)

        
        R0 = Rotation.from_axis_and_angle([0, 1.0, 0], -num * angle, center)
        R = Rotation.from_axis_and_angle([0, 1.0, 0], -angle, center)
        R_left = Rotation.from_axis_and_angle([0, 1.0, 0], -left_angle, center)
        R_right = Rotation.from_axis_and_angle([0, 1.0, 0], right_angle, center)
        bottom = transform_points(bottom, R0)
        bottom_notch = transform_points(bottom,R_left)
        top = transform_points(bottom, R)
        top_notch = transform_points(top, R_right)
        mid_left = [middle_point(bottom_notch[0] , bottom_notch[3]), middle_point(bottom_notch[1], bottom_notch[2])] 
        mid_right = [middle_point(top_notch[0] , top_notch[3]), middle_point(top_notch[1], top_notch[2])]
        
        if(left_angle == 0 and right_angle == 0):
            vertices = bottom + top
            faces = [[0, 1, 2, 3], [7, 6, 5, 4], [3, 7, 4, 0], [6, 2, 1, 5], [7, 3, 2, 6], [5, 1, 0, 4]]
        elif (left_angle == 0):
            vertices = bottom + top + mid_right
            faces = [[0, 1, 2, 3], [3, 7, 8, 0], [8, 4, 0], [9, 6, 2, 1], [1, 5, 9], [7, 3, 2, 6], [5, 1, 0, 4], [4, 8, 9, 5], [8, 9, 6, 7]]
        elif (right_angle == 0):
            vertices = bottom + mid_left + top
            faces = [[0, 1, 5, 4], [2, 3, 4 ,5], [4, 3, 9, 6], [6, 0, 4], [7, 8, 2, 5],[7, 5, 1], [9, 3, 2, 8], [0, 6, 7, 1], [6, 9, 8, 7]]
        else:
            vertices = bottom + mid_left + top + mid_right
            faces = [[0, 1, 5, 4], [2, 3, 4 ,5], [3, 9, 10, 4], [10, 6, 0, 4], [7, 11, 5, 1], [8, 2, 5, 11], [9, 3, 2, 8], [0, 6, 7, 1], [6, 10, 11, 7], [10, 9, 8, 11]]

        mesh = Mesh.from_vertices_and_faces(vertices, faces)

        return mesh
        



if __name__ == '__main__':
    a = CRA_Arch(rise = 5, span = 10, thickness = 0.5, depth = 0.5, n=10)
    print(a.interface())
