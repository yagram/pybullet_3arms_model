# -*-coding:Latin-1 -*
import pybullet as p
import logging
import time
import threading
import utils_collision

logging.basicConfig(level=logging.DEBUG)


class CollisionThread(threading.Thread):

    def __init__(self, world,
                 threshold,
                 group=None,
                 target=None,
                 name=None,
                 args=(),
                 kwargs=None,
                 verbose=None):
        super(CollisionThread, self).__init__() #Constructor of the super class Thread
        self.target = target
        self.name = name
        self.threshold = threshold
        self.world = world
        self.results = []
        self.listCouples = []
        self.positionJoint0Id = p.addUserDebugParameter("Joint0 Pos [deg]", -360, 360, 0) #Create sliders to tune parameters
        self.positionJoint1Id = p.addUserDebugParameter("Joint1 Pos [deg]", -360, 360, 0)
        self.positionJoint2Id = p.addUserDebugParameter("Joint2 Pos [deg]", -180, 180, 0)
        self.positionJoint0 = 0
        self.positionJoint1 = 0
        self.positionJoint2 = 0
        #self.MANUAL_CONTROL = 1

        p.setRealTimeSimulation(1)
        p.setPhysicsEngineParameter(enableFileCaching=0)

        #logging.debug(utils.checkAABBSize(p, self.ppsId, 0))

        return

    def run(self):

        # Prepare World Description -> List formatting
        includeBase = False
        listElems = utils_collision.buildListElems(p)  #Renvoie une liste [body,link] pour le world
        logging.info("Liste des bodyLink" + str(listElems))
        self.listCouples = utils_collision.buildListCoupleLinks(listElems, includeBase)
        logging.info("Liste des paires collisions" + str(self.listCouples))

        """Conserve juste les paires pour lesquelles le calcul de collision est nécessaire (pas de self-coll, pas de base, pas d'importance d'ordre)
        """
        # Declare positions and assign to default values"

        while p.isConnected():
            #MANUAL_CONTROL est un état qui permet le guidage via les boutons ou via des consignes
            """self.positionJoint0 = p.readUserDebugParameter(self.positionJoint0Id) * 6.28 / 360 #Read the input parameters from sliders
            self.positionJoint1 = p.readUserDebugParameter(self.positionJoint1Id) * 6.28 / 360
            self.positionJoint2 = p.readUserDebugParameter(self.positionJoint2Id) * 6.28 / 360

            p.setJointMotorControl2(self.world.ppsId, 1, p.POSITION_CONTROL, targetPosition=self.positionJoint0)
            p.setJointMotorControl2(self.world.ppsId, 2, p.POSITION_CONTROL, targetPosition=self.positionJoint1)
            p.setJointMotorControl2(self.world.ppsId, 3, p.POSITION_CONTROL, targetPosition=self.positionJoint2)"""

            #logging.info("Voici la valeur de positionJoint0: " + str(self.positionJoint0))
            #logging.info("Voici la valeur de positionJoint1: " + str(self.positionJoint1))
            #logging.info("Voici la valeur de positionJoint2: " + str(self.positionJoint2))
            # sleep is NOT required, but no need to compute more than that.
            time.sleep(0.05)

            '''
            Get the closest points between two objects
            '''

            self.results = utils_collision.compute(p, self.listCouples, distanceMin=self.threshold, listOfExclusion=None) #J'ai besoin de chopper le results
            #logging.info("Minimal Distance Computed => " + str(min(self.results[2])))

            ind = self.results[2].index(min(self.results[2]))
            showFrom = self.results[1][ind][0]
            showTo = self.results[1][ind][1]
            for i in range(0,3):
                CoM = p.getLinkState(0, i+1)[0]
                otherPoint = [CoM[0],CoM[1],CoM[2]+100]
                p.addUserDebugLine(lineFromXYZ=CoM, lineToXYZ=otherPoint, lineColorRGB=[0, 0, 1], lineWidth=4,
                                   lifeTime=0.5)
            p.addUserDebugLine(lineFromXYZ=showFrom, lineToXYZ=showTo, lineColorRGB=[1, 1, 0], lineWidth=2,
                               lifeTime=0.5)

        return

    def go_to_target_pos(self, goal):
        self.positionJoint0 = goal[0]
        self.positionJoint1 = goal[1]
        self.positionJoint2 = goal[2]
        return
