#!/usr/bin/env python
# -*- coding: utf-8 -*-

from compas.geometry import Point, Line
from compas.datastructures import Mesh

from compas_assembly.datastructures import Block
from compas_assembly.datastructures import Assembly
from compas_assembly.datastructures import assembly_interfaces_numpy
from compas_assembly.geometry import Arch

from compas_cra.datastructures import CRA_Assembly

from compas_view2.app import App
from compas_view2.objects import Object
from compas_view2.objects import MeshObject

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


if __name__ == '__main__':
    a = CRA_Arch(rise = 5, span = 10, thickness = 0.5, depth = 0.5, n=10)
    print(a.interface())
