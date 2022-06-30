#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Test to evalute arch 
"""


from logging import raiseExceptions


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
    ################################################################
    assembly = CRA_Assembly()
    N = 12
    arch = CRA_Arch(rise = 150, span = 300, thickness = 20, depth = 20, n = N)
    for i, block in enumerate(arch.blocks()):
        assembly.add_block(Block.from_shape(block), key=i)
    
    assembly.set_boundary_conditions([0,N-1])
    
    ################################################################
    #Create Analysis Interfaces
    ################################################################

    ##Method 01#
    # interfaces, keys = arch.interface()
    # for i, interface in enumerate(interfaces):
    #     assembly.add_interfaces_from_meshes([interface], keys[i][0] , keys[i][1])
    
    #Method 02#
    assembly_interfaces_numpy(assembly, nmax=10, amin=1e-1, tmax=1e-6)

    ################################################################
    #Apply Safety Factor
    ################################################################

    safety_factor = 0.6
    result = False
    reduction_number = 0.1
    
    while safety_factor > 0:
        safety_factor -= reduction_number
        try:
            assembly_copy = assembly.copy()
            for edge in assembly_copy.edges():
                interfaces = assembly_copy.edge_attribute(edge, "interfaces")
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
            #Solve and Visualization
            ################################################################
            
            mu = 0.6
            dispbnd = 1e-1
            overlap = 1e-3
            d = 0.15
            
            cra_solve(assembly_copy, verbose=True, density=d, d_bnd=dispbnd, eps=overlap, mu=mu)
            assembly_success = assembly_copy.copy()
            result = True

        except:

            break

        
    if result == True:
        #cra_penalty_solve(assembly, verbose=True, density=d, d_bnd=dispbnd, eps=overlap, mu=mu)
        print("Safety Factor of the Structure is " , "{:5.2f}".format(safety_factor + reduction_number))
        cra_view(assembly_success, resultant=False, nodal=True, grid=False, weights=False,
                displacements=False, dispscale=1, scale=1/10*d)
    else:
        print("Fail to find safety factor; Not safe ;).")