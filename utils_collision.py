import logging
import math
from pytictoc import TicToc

logging.basicConfig(level=logging.DEBUG)

def pretty_print(vector):
    x = round(vector[0],2)
    y = round(vector[1],2)
    z = round(vector[2],2)
    print('('+str(x)+', '+ str(y)+', '+ str(z)+')')



def checkAABBSize(p, bodyId, linkIndex):
    aabb = p.getAABB(bodyId, linkIndex)
    sol = []
    sol.append("\nAABB Size => ")
    sol.append("\ndistance patient X = " + str(aabb[1][0] - aabb[0][0]))
    sol.append("\ndistance patient Y = " + str(aabb[1][1] - aabb[0][1]))
    sol.append("\ndistance patient Z = " + str(aabb[1][2] - aabb[0][2]))
    return ''.join(sol)


"""
drawAABB from https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/examples/getAABB.py
"""
def drawAABB(p, bodyId, linkIndex):
    aabb = p.getAABB(bodyId, linkIndex)
    aabbMin = aabb[0]
    aabbMax = aabb[1]


    #print(aabbMin)
    #print(aabbMax)

    f = [aabbMin[0], aabbMin[1], aabbMin[2]]
    t = [aabbMax[0], aabbMin[1], aabbMin[2]]
    p.addUserDebugLine(f, t, [1, 0, 0])
    f = [aabbMin[0], aabbMin[1], aabbMin[2]]
    t = [aabbMin[0], aabbMax[1], aabbMin[2]]
    p.addUserDebugLine(f, t, [0, 1, 0])
    f = [aabbMin[0], aabbMin[1], aabbMin[2]]
    t = [aabbMin[0], aabbMin[1], aabbMax[2]]
    p.addUserDebugLine(f, t, [0, 0, 1])

    f = [aabbMin[0], aabbMin[1], aabbMax[2]]
    t = [aabbMin[0], aabbMax[1], aabbMax[2]]
    p.addUserDebugLine(f, t, [1, 1, 1])

    f = [aabbMin[0], aabbMin[1], aabbMax[2]]
    t = [aabbMax[0], aabbMin[1], aabbMax[2]]
    p.addUserDebugLine(f, t, [1, 1, 1])

    f = [aabbMax[0], aabbMin[1], aabbMin[2]]
    t = [aabbMax[0], aabbMin[1], aabbMax[2]]
    p.addUserDebugLine(f, t, [1, 1, 1])

    f = [aabbMax[0], aabbMin[1], aabbMin[2]]
    t = [aabbMax[0], aabbMax[1], aabbMin[2]]
    p.addUserDebugLine(f, t, [1, 1, 1])

    f = [aabbMax[0], aabbMax[1], aabbMin[2]]
    t = [aabbMin[0], aabbMax[1], aabbMin[2]]
    p.addUserDebugLine(f, t, [1, 1, 1])

    f = [aabbMin[0], aabbMax[1], aabbMin[2]]
    t = [aabbMin[0], aabbMax[1], aabbMax[2]]
    p.addUserDebugLine(f, t, [1, 1, 1])

    f = [aabbMax[0], aabbMax[1], aabbMax[2]]
    t = [aabbMin[0], aabbMax[1], aabbMax[2]]
    p.addUserDebugLine(f, t, [1.0, 0.5, 0.5])
    f = [aabbMax[0], aabbMax[1], aabbMax[2]]
    t = [aabbMax[0], aabbMin[1], aabbMax[2]]
    p.addUserDebugLine(f, t, [1, 1, 1])
    f = [aabbMax[0], aabbMax[1], aabbMax[2]]
    t = [aabbMax[0], aabbMax[1], aabbMin[2]]
    p.addUserDebugLine(f, t, [1, 1, 1])

    return aabb


def compute(p,listCouples, distanceMin, listOfExclusion=[]):
    """
    compute computes the distances between all Links provided, and return them
    :param p: the pybullet library engine
    :param listCouples: the list of the couples [[bodyA,LinkX],[bodyB, linkY] between which the computation shall be
    done
    :param distanceMin: distance over which there is no computation
    :param listOfExclusion:
    Format = [ [["BodyA", "LinkA"],["BodyB", "LinkB"]] , ...]
    :return: a tuple containing the couple between which the distance was computed, the points (World coordinates)
    between which the distance was computed, and the distance computed
    """
    if listOfExclusion is not None and len(listOfExclusion) > 0:
        raise NotImplementedError

    t_forLoop = TicToc()
    distCouples = []
    lineCouples = []
    # [[[1, 0], [2, 0]]]
    t_forLoop.tic()
    for couple in listCouples:
        bodyA = couple[0][0]
        linkIndexA = couple[0][1]
        bodyB = couple[1][0]
        linkIndexB = couple[1][1]
        closestPointsVec = p.getClosestPoints(bodyA=bodyA,
                                              bodyB=bodyB,
                                              distance=distanceMin,
                                              linkIndexA=linkIndexA,
                                              linkIndexB=linkIndexB)
        #distTemp = 99999
        distTemp = 99999
        fromTemp = (0, 0, 0)
        toTemp = (0, 0, 0)
        if len(closestPointsVec) > 0:
            for c in closestPointsVec:
                distCur = c[8]
                if distCur < distanceMin:
                    distTemp = distCur
                    fromTemp = c[5]
                    toTemp = c[6]

        distCouples.append(distTemp)
        lineCouples.append([fromTemp, toTemp])

    #t_forLoop.toc("TICTOC -> Time elapsed for computing (For-loop) => ")

    assert len(listCouples) is len(distCouples), "The lengths of vectors are not equal"
    assert len(listCouples) is len(lineCouples), "The lengths of vectors are not equal"

    return [listCouples, lineCouples, distCouples]


def buildListElems(p):
    """
    :param p: pybullet wrapper (import pybullet as p)
    :return: a list of the bodyLink elements of the world loaded.
    """
    myList = []
    numBodies = p.getNumBodies()
    for body in range(0, numBodies):
        numJoints = p.getNumJoints(body)
        for link in range(-1, numJoints):
            myList.append([body, link])
    return myList

def buildListCoupleLinks(buildListElems, includeBase=False):
    """
    Build the list of
    from a list [[0, -1], [1, -1], [1, 0], [2, -1], [2, 0]],
    it outputs : [[[1, 0], [2, 0]]],
    => A list containing lists of two links for which the computation needs to take place.
    Assumptions:
    * Elements are self-excluding (no need to compute anything for an element with itself)
    * Base of the elements should not be included in the computation (this can be changed)
    * [a,b] == [b,a] => the order is not important regarding the collision computations

    :param buildListElems: list of the [body,link] existing in the World
    :return: A list containing the lists of two elements of interest
    """
    listCoupleLinks = []
    base = [0, 0]
    for elem in buildListElems:
        for elem2 in buildListElems:
            if elem is elem2:
                continue
            elif elem[0] is elem2[0]: #Si il se trouvent sur le même body alors ils ne sont pas candidats
                continue
            elif elem == base or elem2 == base:
                continue
            elif elem[1] is -1 and not includeBase:
                continue
            elif elem2[1] is -1 and not includeBase:
                continue
            elif [elem2, elem] in listCoupleLinks:
                continue
            else:
                listCoupleLinks.append([elem, elem2])

    return listCoupleLinks


