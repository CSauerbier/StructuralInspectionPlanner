#!/usr/bin/env python

#TO-DO: Naming (instance etc...)
import numpy as np
from SetCoverPy import setcover

from koptplanner.srv import SetCoverSolver, SetCoverSolverResponse
import rospy
from rospy.numpy_msg import numpy_msg   #TO-DO:

##TO-DO: Disable and enable printing, suppress warnings. Remove
import sys, os
import warnings
warnings.filterwarnings("ignore")

# Disable
def blockPrint():
    sys.stdout = open(os.devnull, 'w')

# Restore
def enablePrint():
    sys.stdout = sys.__stdout__

def handle_SetCoverSolver(req):

    #Convert Message into numpy array
    instance_count = req.visibility_matrix.layout.dim[0].size
    candidate_count = req.visibility_matrix.layout.dim[1].size

    print("No tris/VPs before: " + str(instance_count) + "/" + str(candidate_count))

    vismat = np.zeros((instance_count, candidate_count), dtype=bool)

    for inst in range(instance_count):
        for cand in range(candidate_count):
            # vismat[inst, cand] = req.visibility_matrix.data[inst + instance_count*cand]
            vismat[inst, cand] = req.visibility_matrix.data[inst + instance_count*cand]


    #vismattri,vp]
    #find uncovered rows (triangles)
    instances_retained = []
    print("Shape of vector: " + str(vismat[0,:].shape))
    for i in range(instance_count):
        #If at least one entry in the row is True, mark it to be retained
        if(vismat[i,:].sum() != 0):
            instances_retained.append(i)
    #Apply mask, remove uncovered rows
    vismat = vismat[instances_retained, :]

    #TO-DO: Test, remove
    # return SetCoverSolverResponse(instances_retained)

    print("No tris/VPs after: " + str(vismat.shape))

    cost = np.ones(vismat.shape[1])

    # blockPrint()
    g = setcover.SetCover(vismat, cost)
    solution, time_used = g.SolveSCP()
    # enablePrint()

    solution_entries = []
    # print("Sizes Instances retained, g.s: " + str(len(instances_retained)) + "/" + str(len(g.s)))

    for i in range(len(g.s)):  #TO-DO: -1 correct?
        if(g.s[i] == True):
            solution_entries.append(i)

    return SetCoverSolverResponse(solution_entries)
    # #Boolean output array TO-DO:
    # solution_array = np.zeros(candidate_count, dtype=bool)
    # solution_array[solution_entries] = True

    # return SetCoverSolverResponse(solution_array)


def SetCoverSolver_server():
    rospy.init_node('SetCoverSolver_server')
    s = rospy.Service('set_cover_solver', SetCoverSolver, handle_SetCoverSolver)
    rospy.spin()

if __name__ == "__main__":
    SetCoverSolver_server()