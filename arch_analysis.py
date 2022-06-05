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
    N = 30
    arch = CRA_Arch(rise = 5, span = 10, thickness = 1, depth = 0.5, n = N)
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
    assembly_interfaces_numpy(assembly, nmax=10, amin=1e-2, tmax=1e-2)

    ################################################################
    #Solve and Visualization
    ################################################################
       
    mu = 0.9
    dispbnd = 1e-1
    overlap = 1e-3
    d = 1

    #cra_solve(assembly, verbose=True, density=d, d_bnd=dispbnd, eps=overlap, mu=mu)
    cra_penalty_solve(assembly, verbose=True, density=d, d_bnd=dispbnd, eps=overlap, mu=mu)
    cra_view(assembly, resultant=True, nodal=True, grid=False, weights=False,
             displacements=False, dispscale=1, scale=1/d)