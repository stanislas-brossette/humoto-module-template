import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt
import argparse
import collections

np.set_printoptions(threshold=100000000000000)

#Default values
default_height = 0.6
default_nIter = 2
default_radius = 1.0
default_center = [0.0, 0.0, 0.0]

#Argument parsing
parser = argparse.ArgumentParser()
parser.add_argument("--height", help="Height of the half-space",
        type=float, default=default_height)
parser.add_argument("--nIter", help="Number of iteration of the spheres discretization",
        type=int, default=default_nIter)
parser.add_argument("--radius", help="Radius of the sphere",
        type=float, default=default_radius)
parser.add_argument("--center", help="Center of the sphere", nargs=3,
        type=float, default=default_center)
parser.add_argument("--fileName", help="Name of the file where to write the constraint",
        type=str, default='../../example/task_kinematic.h.in')
parser.add_argument("--plot", help="If True, the constraint polygon is plotted",
        type=bool, default='False')
args = parser.parse_args()

flatPolygon = collections.namedtuple('flatPolygon', ['vertices', 'normal', 'center'])
cstr = collections.namedtuple('cstr', ['A', 'b'])

def main(args):
    center = np.array(args.center)
    nIter = args.nIter
    radius = args.radius
    height = args.height

    faces = discretizeSphere(center, radius, nIter)
    spherePolygons = computeNormals(faces, center)
    constraintPolygons = cutSphere(spherePolygons, height, center)
    constraint = polygonToMatrixConstraint(constraintPolygons)
    #writeConstraintToFile(constraint, args.fileName)

    xFoot_ = np.array([  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.6, 0.6, 0.6, 0.6, 0.6, 0.6, 0.6, 0.6, 0.6, 0.6, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1.2, 1.2, 1.2, 1.2, 1.2, 1.2, 1.2, 1.2, 1.2, 1.2, 1.4, 1.4, 1.4, 1.4, 1.4, 1.4, 1.4, 1.4, 1.4, 1.4, 1.6, 1.6, 1.6, 1.6, 1.6, 1.6, 1.6, 1.6, 1.6, 1.6, 1.8, 1.8, 1.8, 1.8, 1.8, 1.8, 1.8, 1.8, 1.8, 1.8, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2])
    yFoot_ = np.array([   0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -0.1, -0.1, -0.1, -0.1, -0.1, -0.1, -0.1, -0.1, -0.1, -0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, -0.1, -0.1, -0.1, -0.1, -0.1, -0.1, -0.1, -0.1, -0.1, -0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, -0.1, -0.1, -0.1, -0.1, -0.1, -0.1, -0.1, -0.1, -0.1, -0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, -0.1, -0.1, -0.1, -0.1, -0.1, -0.1, -0.1, -0.1, -0.1, -0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, -0.1, -0.1, -0.1, -0.1, -0.1, -0.1, -0.1, -0.1, -0.1, -0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, -0.1, -0.1, -0.1, -0.1, -0.1, -0.1, -0.1, -0.1, -0.1, -0.1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
    zFoot_ = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
    x = np.array([   0.1,    0.1,    0.1,    0.1,    0.1,    0.1,    0.1,    0.1,    0.1,    0.1,    0.1,    0.1,    0.1,    0.1,    0.1,    0.1,    0.1,    0.1,    0.1,    0.1,    0.1,    0.1,    0.1,    0.1,    0.1,    0.1, 0.1002, 0.1015, 0.1058, 0.1143, 0.1268, 0.1416, 0.1577, 0.1744, 0.1911, 0.2078, 0.2254, 0.2447, 0.2661, 0.2891, 0.3128, 0.3356, 0.3565, 0.3751, 0.3919,  0.408, 0.4251, 0.4442, 0.4655, 0.4887, 0.5125, 0.5356, 0.5565, 0.5751, 0.5919, 0.6081, 0.6252, 0.6442, 0.6655, 0.6887, 0.7125, 0.7355, 0.7565, 0.7751, 0.7919, 0.8081, 0.8252, 0.8442, 0.8656, 0.8887, 0.9125, 0.9355, 0.9565, 0.9751, 0.9919,  1.008,  1.025,  1.044,  1.066,  1.089,  1.113,  1.136,  1.157,  1.175,  1.192,  1.208,  1.225,  1.244,  1.266,  1.289,  1.313,  1.336,  1.357,  1.375,  1.392,  1.408,  1.425,  1.444,  1.466,  1.489,  1.513,  1.536,  1.557,  1.575,  1.592,  1.608,  1.625,  1.644,  1.666,  1.689,  1.713,  1.736,  1.757,  1.775,  1.792,  1.808,  1.824,  1.842,  1.863,  1.886,  1.911,  1.936,  1.958,  1.976,  1.989,  1.997,  2.001,  2.002,  2.001,  2.001,      2,      2,      2,      2,      2,      2,      2,      2,      2,      2,      2,      2,      2,      2,      2,      2,      2,      2,      2,      2,      2,      2,      2,      2,      2,      2,      2,      2,      2,      2,      2,      2,      2,      2,      2,      2,      2,      2,      2,      2,      2,      2,      2,      2,      2,      2,      2,      2,      2,      2,      2,      2,      2,      2,      2,      2,      2,      2,      2,      2,      2,      2,      2,      2,      2,      2,      2,      2,      2,      2])
    y = np.array([  -0.09944,   -0.09603,   -0.08898,   -0.07786,   -0.06254,   -0.04472,    -0.0273,   -0.01317,  -0.004053,  0.0001645,   0.001011,  0.0004641,  2.546e-05,  0.0001508,  0.0001597,  -0.001448,  -0.006227,   -0.01499,   -0.02749,   -0.04249,   -0.05791,    -0.0713,   -0.08088,   -0.08603,   -0.08687,   -0.08346,   -0.07556,   -0.06264,   -0.04383,   -0.01828,     0.0114,    0.03588,    0.05361,    0.06507,    0.07081,    0.07122,    0.06632,    0.05578,    0.03889,    0.01484,   -0.01393,   -0.03795,   -0.05526,   -0.06634,   -0.07179,   -0.07197,   -0.06688,   -0.05619,   -0.03918,   -0.01503,     0.0138,    0.03786,    0.05519,    0.06629,    0.07175,    0.07193,    0.06686,    0.05617,    0.03917,    0.01502,   -0.01381,   -0.03786,   -0.05519,   -0.06629,   -0.07175,   -0.07194,   -0.06686,   -0.05617,   -0.03917,   -0.01502,    0.01381,    0.03786,    0.05519,    0.06629,    0.07175,    0.07194,    0.06686,    0.05617,    0.03917,    0.01502,   -0.01381,   -0.03786,   -0.05519,   -0.06629,   -0.07175,   -0.07194,   -0.06686,   -0.05617,   -0.03917,   -0.01502,    0.01381,    0.03786,    0.05519,    0.06629,    0.07175,    0.07194,    0.06686,    0.05617,    0.03917,    0.01502,   -0.01381,   -0.03786,   -0.05519,   -0.06629,   -0.07175,   -0.07194,   -0.06686,   -0.05617,   -0.03917,   -0.01502,    0.01381,    0.03782,    0.05498,    0.06588,    0.07114,    0.07108,    0.06571,    0.05467,    0.03719,     0.0124,   -0.01759,   -0.04332,   -0.06225,   -0.07527,   -0.08325,   -0.08673,   -0.08595,   -0.08085,   -0.07129,   -0.05794,   -0.04266,   -0.02782,   -0.01541,  -0.006568,  -0.001513,  0.0004627,  0.0006517,  0.0002817,   1.75e-05, -2.769e-05, -3.959e-06,  2.655e-12, -3.308e-13,  2.567e-14,  7.486e-13,  2.475e-12, -3.982e-12,  2.736e-12, -1.792e-12,  1.196e-12, -1.476e-12,  8.636e-13, -2.539e-12, -2.369e-12,  -3.36e-13,  1.097e-13,  1.044e-12, -1.606e-12, -7.626e-13,  1.363e-12, -1.539e-13,  2.056e-13, -1.508e-12,  -3.61e-13,  -2.25e-12, -2.829e-12, -3.005e-13, -7.962e-13,  4.642e-13, -1.353e-12,  6.791e-13,  3.312e-12, -6.166e-13,   4.15e-14,  2.882e-12, -2.073e-12, -1.403e-12,  2.858e-12, -6.552e-13,  2.003e-12, -3.217e-12, -2.571e-12, -2.602e-12, -6.916e-14, -1.915e-12, -2.053e-12, -1.313e-12,  6.657e-14,  2.639e-13,  3.659e-13,  2.043e-13,  1.048e-12,  6.189e-14,  2.034e-13, -1.148e-12,  1.342e-12,  2.148e-12,  1.208e-12, -1.851e-14,   9.27e-13])
    z = np.array([0.8031, 0.8215, 0.8553, 0.8955, 0.9317, 0.9575, 0.9717, 0.9775, 0.9793, 0.9798, 0.9799,   0.98, 0.9801, 0.9801, 0.9801, 0.9798, 0.9789, 0.9767, 0.9738, 0.9717, 0.9717, 0.9737,  0.976, 0.9773, 0.9775, 0.9765, 0.9734, 0.9683,  0.963,  0.961, 0.9645,  0.972, 0.9807,  0.988, 0.9924, 0.9927, 0.9882, 0.9802, 0.9712, 0.9653, 0.9655, 0.9717, 0.9805, 0.9883, 0.9928, 0.9928, 0.9884, 0.9805, 0.9715, 0.9655, 0.9655, 0.9715, 0.9804, 0.9883, 0.9928, 0.9928, 0.9884, 0.9805, 0.9715, 0.9655, 0.9655, 0.9715, 0.9804, 0.9883, 0.9928, 0.9928, 0.9884, 0.9805, 0.9715, 0.9655, 0.9655, 0.9715, 0.9804, 0.9883, 0.9928, 0.9928, 0.9884, 0.9805, 0.9715, 0.9655, 0.9655, 0.9715, 0.9804, 0.9883, 0.9928, 0.9928, 0.9884, 0.9805, 0.9715, 0.9655, 0.9655, 0.9715, 0.9804, 0.9883, 0.9928, 0.9928, 0.9884, 0.9805, 0.9715, 0.9655, 0.9655, 0.9715, 0.9804, 0.9883, 0.9928, 0.9928, 0.9884, 0.9805, 0.9715, 0.9655, 0.9655, 0.9715, 0.9804, 0.9883, 0.9927, 0.9927, 0.9882, 0.9801, 0.9712, 0.9654, 0.9659, 0.9724, 0.9814, 0.9894, 0.9945, 0.9968,  0.997, 0.9956, 0.9934, 0.9915, 0.9915, 0.9936, 0.9965, 0.9987, 0.9997, 0.9999, 0.9999, 0.9999,      1,      1,      1,      1,      1,      1,      1,      1,      1,      1,      1,      1,      1,      1,      1,      1,      1,      1,      1,      1,      1,      1,      1,      1,      1,      1,      1,      1,      1,      1,      1,      1,      1,      1,      1,      1,      1,      1,      1,      1,      1,      1,      1,      1,      1,      1,      1,      1,      1,      1,      1,      1,      1,      1,      1,      1,      1,      1,      1,      1,      1,      1])

    if args.plot:
        fig = plt.figure()
        ax = fig.gca(projection='3d')
        ax = addFacesToPlot(ax, constraintPolygons)
        ax = addNormalsToPlot(ax, constraintPolygons)
        ax = checkPoints(ax, x - xFoot_, y - yFoot_, z - zFoot_, constraintPolygons)
        #ax = addRandomPointsCheck(ax, constraintPolygons, 1000)
        ax.set_xlim(-1.0, 1.0)
        ax.set_ylim(-1.0, 1.0)
        ax.set_zlim(-1.0, 1.0)
        plt.show()

    return

def writeConstraintToFile(constraint, fileName):
    A = constraint.A
    b = constraint.b
    myFile = open(fileName, 'w')
    myFile.write('#pragma once\n')
    myFile.write('#include <Eigen/Core>\n')
    myFile.write('class GeneratedKinematicConstraint\n')
    myFile.write('{\n')
    myFile.write('  public:\n')
    myFile.write('    Eigen::MatrixXd A;\n')
    myFile.write('    Eigen::VectorXd b;\n')
    myFile.write('    GeneratedKinematicConstraint()\n')
    myFile.write('    {\n')
    myFile.write('      A.resize(')
    myFile.write(str(A.shape[0]))
    myFile.write(',')
    myFile.write(str(A.shape[1]))
    myFile.write(');\n')
    myFile.write('      b.resize(')
    myFile.write(str(b.shape[0]))
    myFile.write(');\n')
    myFile.write('      A <<')
    for i in np.arange(A.shape[0]):
        for j in np.arange(A.shape[1]):
            myFile.write(str(A[i,j]))
            if not (i == A.shape[0]-1 and j == A.shape[1]-1):
                myFile.write(', ')
    myFile.write(';\n')
    myFile.write('      b <<')
    for i in np.arange(b.shape[0]):
        myFile.write(str(b[i,0]))
        if i != b.shape[0]-1:
            myFile.write(', ')
    myFile.write(';\n')
    myFile.write('}\n};\n')

#Computes A and b such that b<Ax if the x is inside the polytope
def polygonToMatrixConstraint(polygon):
    A = np.zeros((len(polygon),3))
    b = np.zeros((len(polygon),1))
    for i in np.arange(len(polygon)):
        A[i,:] = polygon[i].normal
        b[i,0] = np.dot(polygon[i].normal, polygon[i].center)
    cstrPolytop = cstr(A,b)
    return cstrPolytop
        
def checkPoints(ax, x, y, z, constraintPolygons):
    pointsInside = []
    pointsOutside = []
    for i in np.arange(x.shape[0]):
        point = np.array([x[i],y[i],z[i]])
        if checkPointInside(constraintPolygons, point):
            pointsInside.append(point)
        else:
            pointsOutside.append(point)
    pointsInside = np.array(pointsInside)
    pointsOutside = np.array(pointsOutside)

    ax.scatter(pointsInside[:,0],pointsInside[:,1],pointsInside[:,2], c='b', marker='x')
    ax.scatter(pointsOutside[:,0],pointsOutside[:,1],pointsOutside[:,2], c='r', marker='x')
    return ax

def addRandomPointsCheck(ax, constraintPolygons, nPoints):
    randPoints = 2 * np.random.rand(100,3) - 1.0
    pointsInside = []
    pointsOutside = []
    for i in np.arange(randPoints.shape[0]):
        point = np.array([randPoints[i,0],randPoints[i,1],randPoints[i,2]])
        if checkPointInside(constraintPolygons, randPoints[i,:]):
            pointsInside.append(randPoints[i,:])
        else:
            pointsOutside.append(randPoints[i,:])
    pointsInside = np.array(pointsInside)
    pointsOutside = np.array(pointsOutside)

    ax.scatter(pointsInside[:,0],pointsInside[:,1],pointsInside[:,2], c='b', marker='x')
    ax.scatter(pointsOutside[:,0],pointsOutside[:,1],pointsOutside[:,2], c='r', marker='x')
    return ax

#This function should be used only on a complete sphere, not a cut one
def computeNormals(sphereFaces, sphereCenter):
    newFaces = []
    for face in sphereFaces:
        faceCenter = computeFaceCenter(face)
        normal = np.cross(face.vertices[1] - face.vertices[0], face.vertices[2] - face.vertices[1])
        normal = normal/np.linalg.norm(normal)
        if(np.dot(normal, sphereCenter - faceCenter) <= 0):
            normal = -normal
        newFaces.append(flatPolygon(face.vertices, normal, faceCenter))
    return newFaces

def checkPointInside(faces, point):
    for face in faces:
        if np.dot(face.normal, point - face.center)<0:
            return False
    return True

def discretizeSphere(center, radius, nIter):
    pXp = center + radius * np.array([1.0, 0.0, 0.0])
    pXm = center + radius * np.array([-1.0, 0.0, 0.0])
    pYp = center + radius * np.array([0.0, 1.0, 0.0])
    pYm = center + radius * np.array([0.0, -1.0, 0.0])
    pZp = center + radius * np.array([0.0, 0.0, 1.0])
    pZm = center + radius * np.array([0.0, 0.0, -1.0])
    faces = []
    faces.append(flatPolygon([pXp, pYp, pZp],[],[]))
    faces.append(flatPolygon([pXp, pYm, pZp],[],[]))
    faces.append(flatPolygon([pXp, pYp, pZm],[],[]))
    faces.append(flatPolygon([pXp, pYm, pZm],[],[]))
    faces.append(flatPolygon([pXm, pYp, pZp],[],[]))
    faces.append(flatPolygon([pXm, pYm, pZp],[],[]))
    faces.append(flatPolygon([pXm, pYp, pZm],[],[]))
    faces.append(flatPolygon([pXm, pYm, pZm],[],[]))
    iter = 0
    while iter < nIter:
        newFaces = []
        for face in faces:
            m01 = (face.vertices[0] + face.vertices[1])/2.0
            m12 = (face.vertices[1] + face.vertices[2])/2.0
            m20 = (face.vertices[2] + face.vertices[0])/2.0
            m01 = center + radius * (m01-center)/np.linalg.norm(m01-center)
            m12 = center + radius * (m12-center)/np.linalg.norm(m12-center)
            m20 = center + radius * (m20-center)/np.linalg.norm(m20-center)
            newFaces.append(flatPolygon([m01, m20, face.vertices[0]],[],[]))
            newFaces.append(flatPolygon([m12, m01, face.vertices[1]],[],[]))
            newFaces.append(flatPolygon([m20, m12, face.vertices[2]],[],[]))
            newFaces.append(flatPolygon([m01, m12, m20],[],[]))
        faces = newFaces
        iter = iter + 1
    return faces

def computeFaceCenter(face):
    center = np.array([0,0,0])
    for vertex in face.vertices:
        center = center + vertex
    center = center/len(face.vertices)
    return center

def cutSphere(faces, height, sphereCenter):
    newFaces = []
    for face in faces:
        if (face.vertices[0][2]>height and face.vertices[1][2]>height and face.vertices[2][2]>height):
            newFaces.append(face)
        elif (face.vertices[0][2]>height or face.vertices[1][2]>height or face.vertices[2][2]>height):
            cutFace = computeCutFace(face, height)
            cutFaceCenter = computeFaceCenter(cutFace)
            newFaces.append(flatPolygon(cutFace.vertices, face.normal, cutFaceCenter))
    bottomFaceNormal = np.array([0,0,1])
    bottomFaceCenter = np.array([sphereCenter[0], sphereCenter[1], height])
    newFaces.append(flatPolygon([], bottomFaceNormal, bottomFaceCenter))
    return newFaces

def cutSegment(p0, p1, height):
    if(p0[2]>height and p1[2]>height):
        return [p0, p1]
    elif(p0[2]>height or p1[2]>height):
        pUp = []
        pDown = []
        if(p0[2]>height):
            pUp = p0
            pDown = p1
        if(p1[2]>height):
            pUp = p1
            pDown = p0
        alpha = (height - pDown[2])/(pUp[2] - pDown[2])
        pMid = pDown + alpha*(pUp - pDown)
        return [pMid, pUp]

def computeCutFace(face, height):
    newFaceVertices = []
    nPointsAbove = 0
    pointsAbove = []
    pointsBelow = []
    for vertex in face.vertices:
        if(vertex[2]>height):
            nPointsAbove = nPointsAbove + 1
            pointsAbove.append(vertex)
        else:
            pointsBelow.append(vertex)
        
    if(nPointsAbove == 3):
        return face
    elif(nPointsAbove == 1):
        seg0 = cutSegment(pointsAbove[0],pointsBelow[0],height)
        seg1 = cutSegment(pointsAbove[0],pointsBelow[1],height)
        newFaceVertices.append(pointsAbove[0])
        newFaceVertices.append(seg0[0])
        newFaceVertices.append(seg1[0])
    elif(nPointsAbove == 2):
        seg0 = cutSegment(pointsAbove[0],pointsBelow[0],height)
        seg1 = cutSegment(pointsAbove[1],pointsBelow[0],height)
        newFaceVertices.append(pointsAbove[0])
        newFaceVertices.append(pointsAbove[1])
        newFaceVertices.append(seg1[0])
        newFaceVertices.append(seg0[0])
    return flatPolygon(newFaceVertices, face.normal, face.center)

def addFacesToPlot(ax, faces):
    for face in faces:
        if len(face.vertices) > 0:
            lineX = np.array([])
            lineY = np.array([])
            lineZ = np.array([])
            for vertex in face.vertices:
                lineX = np.append(lineX, vertex[0])
                lineY = np.append(lineY, vertex[1])
                lineZ = np.append(lineZ, vertex[2])
            lineX = np.append(lineX, face.vertices[0][0])
            lineY = np.append(lineY, face.vertices[0][1])
            lineZ = np.append(lineZ, face.vertices[0][2])
            ax.plot(lineX, lineY, lineZ)
    return ax

def addNormalsToPlot(ax, faces):
    for face in faces:
        lineX = np.array([face.center[0],face.center[0]+0.1*face.normal[0]])
        lineY = np.array([face.center[1],face.center[1]+0.1*face.normal[1]])
        lineZ = np.array([face.center[2],face.center[2]+0.1*face.normal[2]])
        ax.plot(lineX, lineY, lineZ)
    return ax

def plotFaces(faces):
    fig = plt.figure()
    ax = fig.gca(projection='3d')
    ax = addFacesToPlot(ax, faces)
    ax.set_xlim(-1.0, 1.0)
    ax.set_ylim(-1.0, 1.0)
    ax.set_zlim(-1.0, 1.0)
    plt.show()

def plotFacesAndNormals(faces):
    fig = plt.figure()
    ax = fig.gca(projection='3d')
    ax = addFacesToPlot(ax, faces)
    ax = addNormalsToPlot(ax, faces)
    ax.set_xlim(-1.0, 1.0)
    ax.set_ylim(-1.0, 1.0)
    ax.set_zlim(-1.0, 1.0)
    plt.show()






main(args)

