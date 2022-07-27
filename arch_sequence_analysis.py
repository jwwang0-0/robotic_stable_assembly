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
    successful = []


    for i, block in enumerate(arch.blocks()):
        
        
        block2 = block.transformed(s0)
        assembly.add_block(Block.from_shape(block2), key=i)
        analysis, anchors  = arch.sequence(i)
        if analysis == True:
            #temproary method, need to be updated 
            assembly.node_attribute(i-1,"is_support", False)
            assembly.set_boundary_conditions(anchors)
    
            ################################################################
            #Create Analysis Interfaces
            ################################################################

            ##Method 01#
            # interfaces, keys = arch.interface()
            # for i, interface in enumerate(interfaces):
            #     assembly.add_interfaces_from_meshes([interface], keys[i][0] , keys[i][1])
            
            #Method 02#
            assembly_interfaces_numpy(assembly, nmax=10, amin=1e-2, tmax=1e-6)

            ################################################################
            #Apply Safety Factor
            ################################################################
            
            # safety_factor = 1

            # for edge in assembly.edges():
            #     interfaces = assembly.edge_attribute(edge, "interfaces")
            #     for interface in interfaces:

            #         #Update the area of the interface
            #         interface.size = safety_factor * interface.size

            #         #Update points of the interface
            #         pl = Polygon(interface.points)
            #         center = pl.centroid
            #         s = Scale.from_factors([mt.pow(safety_factor,0.5)]*3, Frame(center,[1,0,0],[0,1,0]))
            #         pl_scaled = pl.transformed(s)
            #         interface.points = pl_scaled.points

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
            try:
                cra_solve(assembly, verbose=True, density=d, d_bnd=dispbnd, eps=overlap, mu=mu)
                successful.append(i)
            except:
                print("It fails at ", i, "step")
                break
            #cra_penalty_solve(assembly, verbose=True, density=d, d_bnd=dispbnd, eps=overlap, mu=mu)
            # cra_view(assembly, resultant=False, nodal=True, grid=False, weights=False,
            #         displacements=False, dispscale=1, scale=500*d)
    print(successful)