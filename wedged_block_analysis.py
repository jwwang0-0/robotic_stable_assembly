#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Test to evalute arch 
"""


if __name__ == '__main__':

    import math as mt

    from compas.datastructures import Mesh
    from compas.geometry import Box
    from compas.geometry import Frame
    from compas.geometry import Translation
    from compas.geometry import Polygon, centroid_polygon
    from compas.geometry import Scale
    from compas_assembly.datastructures import Block
    from compas_cra.datastructures import CRA_Assembly
    from compas_cra.datastructures import assembly_interfaces_numpy
    from compas_cra.equilibrium import cra_solve, cra_penalty_solve
    from compas_cra.viewers import cra_view
    

    from arch import CRA_Arch
    from math import radians

    ################################################################
    #Create Geometry
    #Scale to the Requested Range (5*5*5)
    ################################################################
    assembly = CRA_Assembly()
    N = 12
    arch = CRA_Arch(rise = 150, span = 300, thickness = 20, depth = 20, n = N)
    box = arch.bounding_box()
    scale_base = max(box.xsize, box.ysize, box.zsize)
    scale_to = 5
    scale_factor = scale_to / scale_base
    s0 = Scale.from_factors([scale_factor] * 3)
    wedge_angle = 2

    for i, block in enumerate(arch.blocks()):

        if i == 0:
            wedge_block = arch.wedged_block(i,0,radians(wedge_angle))
        elif i==N-1:
            wedge_block = arch.wedged_block(i,0,0)
        elif i== N-2:
            wedge_block = arch.wedged_block(i,0,0)
        elif i==N-3:
            wedge_block = arch.wedged_block(i,radians(-wedge_angle),0)
        else:
            wedge_block = arch.wedged_block(i,radians(-wedge_angle),radians(wedge_angle))
        block2 = wedge_block.transformed(s0)
        assembly.add_block(Block.from_shape(block2), key=i)
    
    assembly.set_boundary_conditions([0,N-1])
    
    ################################################################
    #Create Analysis Interfaces
    ################################################################

    ##Method 01#
    # interfaces, keys = arch.interface()
    # for i, interface in enumerate(interfaces):
    #     assembly.add_interfaces_from_meshes([interface], keys[i][0] , keys[i][1])
    
    #Method 02#
    assembly_interfaces_numpy(assembly, nmax=10, amin=1e-3, tmax=1e-6)

    ################################################################
    #Apply Safety Factor
    ################################################################
    
    safety_factor = 0.4

    for edge in assembly.edges():
        interfaces = assembly.edge_attribute(edge, "interfaces")
        for interface in interfaces:
            #Update the area of the interface
            interface.size = safety_factor * interface.size

            #Update points of the interface
            pl = Polygon(interface.points)
            center = pl.centroid
            s = Scale.from_factors([mt.pow(safety_factor,0.5)]*3, Frame(center,[1,0,0],[0,1,0]))
            pl_scaled = pl.transformed(s)
            interface.points = pl_scaled.points

    ################################################################
    #Data Storage
    ################################################################
    
    #print(assembly.data)
    #assembly.to_json("D:/Jingwen/cra/test.json", pretty = True)



    ################################################################
    #Solve and Visualization
    ################################################################
       
    mu = 0.9
    dispbnd = 1e-1
    overlap = 1e-3
    d = 1
    cra_solve(assembly, verbose=True, density=d, d_bnd=dispbnd, eps=overlap, mu=mu)
    #cra_penalty_solve(assembly, verbose=True, density=d, d_bnd=dispbnd, eps=overlap, mu=mu)
    cra_view(assembly, resultant=False, nodal=True, grid=False, weights=False, edge = False,
        displacements=False, dispscale=1, scale=10*d)
    
    
    #cra_penalty_solve(assembly, verbose=True, density=d, d_bnd=dispbnd, eps=overlap, mu=mu)
