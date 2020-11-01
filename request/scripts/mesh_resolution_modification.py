#!/usr/bin/env python

"""
Remesh the input mesh to remove degeneracies and improve triangle quality.
"""

import argparse
import numpy as np
from numpy.linalg import norm

import pymesh

import rospy
import rospkg
from request.srv import MeshResolutionModification, MeshResolutionModificationResponse
from decimal import Decimal
import os.path


def fix_mesh(mesh, resolution):
    bbox_min, bbox_max = mesh.bbox
    diag_len = norm(bbox_max - bbox_min)

    target_len = diag_len * resolution

    rospy.loginfo("\tTarget resolution: {} mm".format(target_len))

    count = 0
    mesh, __ = pymesh.remove_degenerated_triangles(mesh, 100)
    mesh, __ = pymesh.split_long_edges(mesh, target_len)
    num_vertices = mesh.num_vertices
    while True:
        mesh, __ = pymesh.collapse_short_edges(mesh, 1e-6)
        mesh, __ = pymesh.collapse_short_edges(mesh, target_len,
                                               preserve_feature=True)
        mesh, __ = pymesh.remove_obtuse_triangles(mesh, 150.0, 100)
        if mesh.num_vertices == num_vertices:
            break

        num_vertices = mesh.num_vertices
        rospy.loginfo("\t#vertices: {}".format(num_vertices))
        count += 1
        if count > 2: break

    mesh = pymesh.resolve_self_intersection(mesh)
    mesh, __ = pymesh.remove_duplicated_faces(mesh)
    mesh = pymesh.compute_outer_hull(mesh)
    mesh, __ = pymesh.remove_duplicated_faces(mesh)
    mesh, __ = pymesh.remove_obtuse_triangles(mesh, 179.0, 5)
    mesh, __ = pymesh.remove_isolated_vertices(mesh)

    return mesh


def old_fix_mesh(vertices, faces, detail="normal"):
    bbox_min = np.amin(vertices, axis=0)
    bbox_max = np.amax(vertices, axis=0)
    diag_len = norm(bbox_max - bbox_min)
    if detail == "normal":
        target_len = diag_len * 5e-3
    elif detail == "high":
        target_len = diag_len * 2.5e-3
    elif detail == "low":
        target_len = diag_len * 1e-2
    print("Target resolution: {} mm".format(target_len))

    count = 0
    vertices, faces = pymesh.split_long_edges(vertices, faces, target_len)
    num_vertices = len(vertices)
    while True:
        vertices, faces = pymesh.collapse_short_edges(vertices, faces, 1e-6)
        vertices, faces = pymesh.collapse_short_edges(vertices, faces,
                                                      target_len, preserve_feature=True)
        vertices, faces = pymesh.remove_obtuse_triangles(vertices, faces, 150.0, 100)
        if num_vertices == len(vertices):
            break
        num_vertices = len(vertices)
        print("#v: {}".format(num_vertices))
        count += 1
        if count > 10: break

    vertices, faces = pymesh.resolve_self_intersection(vertices, faces)
    vertices, faces = pymesh.remove_duplicated_faces(vertices, faces)
    vertices, faces, _ = pymesh.compute_outer_hull(vertices, faces, False)
    vertices, faces = pymesh.remove_duplicated_faces(vertices, faces)
    vertices, faces = pymesh.remove_obtuse_triangles(vertices, faces, 179.0, 5)
    vertices, faces, voxels = pymesh.remove_isolated_vertices(vertices, faces)
    return vertices, faces



def handle_mesh_resolution_modification(req):
    

    (path_in, filename_in) = os.path.split(req.file_path_in)
    resolution = req.target_resolution
    filename_out = filename_in.replace('.stl', '') + "_" + '%.1E' % Decimal(resolution) + "_ASCII.stl"

    path_out = str(rospkg.RosPack().get_path('request')) + "/meshes/cache/"
    
    mesh = pymesh.meshio.load_mesh(path_in + "/" + filename_in)
    if(not os.path.isfile(path_out + filename_out)):
        mesh = fix_mesh(mesh, resolution)
        pymesh.meshio.save_mesh(path_out + filename_out, mesh, ascii=True)
        rospy.loginfo("New mesh generated: " + path_out+filename_out)
    else:
        rospy.loginfo("Using cached file")

    return MeshResolutionModificationResponse(path_out+filename_out)

def mesh_resolution_modification_server():
    rospy.init_node('mesh_resolution_modification_server')
    s = rospy.Service('/mesh_resolution_modification', MeshResolutionModification, handle_mesh_resolution_modification)
    rospy.spin()



if __name__ == "__main__":
    mesh_resolution_modification_server()
