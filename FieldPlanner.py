# -*-coding:Latin-1 -*
import pybullet as p
import logging
import time
import threading
from collisionThread import *
from numpy import ndarray
import numpy as np
import math
from typing import List
import sys
import xlsxwriter
import utils_collision
import register
logging.basicConfig(level=logging.DEBUG)

MAX_SPEED = 6*3.1415/180 #Medical speed
MAX_TORQUE = 0.96*171*9.2 #efficiency*gear ratio * motor torque
class FieldPlanner(threading.Thread):

    def __init__(self,
                 goal, world,
                 collisionThread,
                 group=None,
                 target=None,
                 name=None,
                 args=(),
                 kwargs=None,
                 verbose=None):
        super(FieldPlanner, self).__init__()  # Constructor of the super class Thread
        self.target = target
        self.name = name
        self.goal = goal
        self.world = world
        self.collisionThread = collisionThread
        self.movingJoints = self.get_moving_joints(self.world.ppsId)
        self.dofLeoni = self.get_dof(self.world.ppsId)
        self.eta = 0.1
        self.zeta = [15, 15, 100]
        self.rho0 = 0.11
        self.d = 2
        self.alpha0 = 0.001
        self.alphai = 0.01
        self.posBlocage=0
        self.tour=0
        self.sign=1
        self.collision=False
        self.arrayAttLeoni: List[ndarray] = []
        self.arrayAttObs: List[ndarray] = []
        basisList=[[0, 0, 0], [0, 0, 0], 100, "Name"]
        self.arrayRepLeoni = [basisList[:] for i in range(p.getNumJoints(self.world.ppsId)-1)] # Il est nécessaire d'initialiser la liste
        self.arrayRepObs = [basisList[:] for i in range(p.getNumJoints(self.world.myObstacle))]
        emptyJac = [[0 for i in range(self.dofLeoni)] for y in range(3)]
        emptyForce = [0 for i in range(self.dofLeoni)]
        self.arrayJacAtt = [emptyJac[:] for i in range(self.dofLeoni)]
        self.arrayJacRep = [emptyJac[:] for i in range(self.dofLeoni)]
        self.jointPos = [0 for i in range(0, self.dofLeoni)]
        self.jointFrames=[[[0 for i in range(3)],[0 for i in range(4)]]for i in range(p.getNumJoints(self.world.ppsId))]
        self.arrayRepFields: List[ndarray] = []
        self.arrayAttFields: List[ndarray] = []
        self.vectorPos: List[ndarray] = []
        self.arrayRepForces: List[ndarray] = []
        self.arrayAttForces: List[ndarray] = []
        self.jointAxis=[[0 for i in range(3)] for i in range(3)]
        self.jointTorques= np.zeros(3)
        self.goalReached=False
        self.accuracy=0.00005
        self.excelCspace = xlsxwriter.Workbook('data_memoire/traj_min_local.xlsx')
        self.sheetExcelCspace = self.excelCspace.add_worksheet()
        self.excelQST =xlsxwriter.Workbook('data_memoire/torques_obs_up.xlsx')
        self.sheetExcelQST= self.excelQST.add_worksheet()
        self.excelForces =xlsxwriter.Workbook('data_memoire/Allforces_obs_up.xlsx')
        self.sheetExcelForces= self.excelForces.add_worksheet()
        self.excelCoMInfo =xlsxwriter.Workbook('data_memoire/CoMInfo_obs_up_retest.xlsx')
        self.sheetExcelCoMInfo= self.excelCoMInfo.add_worksheet()
        self.qFinal=[]
        # self.qFinal: List[ndarray] = [np.asarray([-0.42133958,-2.30351515,-0.95]),np.asarray([-0.94041358,-2.67598867,-0.62])
        #      ,np.asarray([-1.20294045,-2.11287827,-0.62])] #Pour le goal (40,75,0)
        self.qFinal: List[ndarray] = [np.asarray([0.044958921717760336, -2.498159370400595, -0.95]), np.asarray([0.14421056063219984, -3.1652997779121357, -0.6199999999999999])
              , np.asarray([0.7172773896099363, -3.405316625444558, -0.6199999999999999])]  # Pour le goal (94.737,-117.474,0)

        return

    def run(self):
        # Le but de ce thread va être de calculer la séquence de position que doit effectuer nos objets
        time.sleep(1)  # On attends une petite seconde le temps que l'autre thread commence à run
        #elf.get_final_state()
        row=0
        #self.go_to_first_collision()
        #self.write_Cspace(row)
        #self.write_QST(row)
        #p.resetDebugVisualizerCamera(3, 0, 0,[0,-1.95,-1])
        #self.write_forces(row)
        #self.write_CoMInfo(row)
        row +=1
        substep=0
        #self.construct_C_space()
        while p.isConnected():
            #print("trollu")
            #self.construct_C_space()
            self.update_arrays_att()
            #self.print_array_att_leoni()
            self.update_arrays_rep()
            self.get_att_fields()
            self.get_rep_fields()
            #self.print_array_rep_leoni()
            self.update_jacobians()
            #self.print_array_jac_att()
            #self.print_array_jac_rep()
            self.get_world_rep_forces()
            self.get_world_att_forces()
            #self.print_rep_forces()
            #self.print_att_forces()
            self.proj_world_forces()
            self.step_forward()
            if row//5 > substep:
                substep +=1
                #self.write_Cspace(substep)
                #self.write_QST(substep)
                #self.write_CoMInfo(substep)
                #self.write_forces(substep)
            #self.print_joint_torques()
            #self.print_joint_torques(jointTorques)
            #self.print_joint_pos_deg()
            self.check_accuracy()
            self.reinit_arrays_rep() #Réinitialisation de la distance
            self.clear_arrays_forces()
            row +=1
            #self.print_position_all_link(self.world.ppsId)
        print("finito")

        return

    def update_arrays_att(self):
        """ Update of arrayAttLeoni[]
        arrayAttLeoni[i]: CoM position of i-th link in world coordinates
        Note: arrayAttObs is useless since it is static """
        self.clear_arrays_att() #On clear les listes
        for i in range(1, p.getNumJoints(self.world.ppsId)): #Starting at 1 to skip the Base
            linkState = p.getLinkState(self.world.ppsId, linkIndex=i)
            self.arrayAttLeoni.append(linkState[0]) #Selecting the CoM of link i
        for i in range(0, p.getNumJoints(self.world.myObstacle)):
            linkState = p.getLinkState(self.world.myObstacle, linkIndex=i)
            self.arrayAttObs.append(linkState[0])

        return

    def update_arrays_rep(self):
        """ Update arrayRepLeoni[][] and arrayRepObs[][] using information from collisionThread checking for collision.
        For each link, the shortest distance is computed w.r.t. all obstacles and smallest value is kept for arrayRepLeoni.
        Same is done the other way around for arrayRepObs.
        arrayRepLeoni[i][0]: position of repulsive point on the i-th link (world coordinates)
        arrayRepLeoni[i][1]: position of the corresponding point on the closest obstacle (world coordinates)
        arrayRepLeoni[i][2]: distance of i-th link w.r.t. the closest obstacle
        arrayRepLeoni[i][3]: name of the closest obstacle"""
        tempList = []
        listCouples = self.collisionThread.listCouples #Array with pairs of relevant links for distance computation
        results = self.collisionThread.results #Result from thread
        for i in range(0, len(listCouples)):
            ind = listCouples[i][0][1]-1 #index for arrayRepLeoni, -1 for the Base
            myJointInfo = p.getJointInfo(self.world.myObstacle, listCouples[i][1][1])
            if results[2][i] < self.arrayRepLeoni[ind][2]:
                self.arrayRepLeoni[ind][0] = np.asarray(results[1][i][0])
                self.arrayRepLeoni[ind][1] = np.asarray(results[1][i][1])
                self.arrayRepLeoni[ind][2] = results[2][i]
                self.arrayRepLeoni[ind][3] = myJointInfo[12] #Obstacle name
            ind = listCouples[i][1][1] #index for arrayRepObs
            myJointInfo = p.getJointInfo(self.world.ppsId, listCouples[i][0][1])
            if results[2][i] < self.arrayRepObs[ind][2]:
                self.arrayRepObs[ind][0] = np.asarray(results[1][i][1]) #Inversion des vecteurs
                self.arrayRepObs[ind][1] = np.asarray(results[1][i][0])
                self.arrayRepObs[ind][2] = results[2][i]
                self.arrayRepObs[ind][3] = myJointInfo[12]  # link name
        return

    def reinit_arrays_rep(self):
        for i in range(0, len(self.arrayRepLeoni)):
            self.arrayRepLeoni[i][2] = 100 #La distance doit être réinitialisée
        for i in range(0, len(self.arrayRepObs)):
            self.arrayRepObs[i][2] = 100
        return

    def clear_arrays_att(self):
        self.arrayAttLeoni.clear()
        self.arrayAttObs.clear()
        return

    def clear_arrays_forces(self):
        self.arrayRepForces.clear()
        self.arrayAttForces.clear()
        return

    def CoM_to_point(self, indexBody, worldPoint) -> tuple:
        """localPoint est un vecteur allant du CoM au point de répulsion
        À vérifier"""
        localPoint = tuple(np.subtract(worldPoint, self.jointFrames[indexBody][0])) #vecteur coordonnées absolues à projeter
        linkState=p.getLinkState(self.world.ppsId,indexBody) #Ici j'avais mis -1 puis jl'ai retiré et ca a fonctionné
        #Correspond à la transformation du world vers la frame locale.
        worldToFrame = p.getMatrixFromQuaternion(linkState[5])#On transforme le quaternion en matrice de rotation
        worldToFrame = np.asarray(worldToFrame)
        localPoint = np.asarray(localPoint) #vecteur entre CoM et point de répulsion à projeter dans la frame locale
        worldToFrame = np.reshape(worldToFrame, (3, 3))
        localPoint = tuple(np.matmul(localPoint,worldToFrame)) #projection dans la frame locale
        return localPoint

    def frame_to_CoM(self, indexBody) -> tuple:
        """Permet de récupérer l'attribut localInertialFramePosition
        valeur xyz de "inertial" dans l'urdf"""
        localPoint = p.getLinkState(self.world.ppsId, indexBody)[2]
        return localPoint

    def update_joint_pos(self, indexBody):
        """"Met à jour la liste contenant la position des joints(sans la base)
        On a besoin des joints intérmédiaires pour la matrice jacobienne dans tous les cas
        doit avoir la même taille que dof
        """
        for joint in range(0, self.dofLeoni):
            self.jointPos[joint] = p.getJointState(indexBody, joint+1)[0] #+1 pour éviter la base

        return

    def update_joint_frame(self):
        """ Update jointFrames[][] array:
        jointFrames[i][0]: position of the i-th joint (in world coordinates).
        jointFrames[i][1]: quaternion from world to i-th joint frame.
        ."""
        self.jointFrames[0][0]=np.asarray(p.getJointInfo(self.world.ppsId,0)[14]) #First joint is at parentFramePos
        self.jointFrames[0][1]=np.asarray(p.getJointInfo(self.world.ppsId,1)[15])
        for i in range(1, p.getNumJoints(self.world.ppsId)):
            relJointPos=np.asarray(p.getJointInfo(self.world.ppsId,i)[14]) #jointPos relative to parentLink (in parentLink's frame)
            linkState=p.getLinkState(self.world.ppsId,i-1)
            frameToWorld=np.reshape(np.asarray(p.getMatrixFromQuaternion(p.invertTransform(linkState[0],linkState[1])[1])),(3,3))
            #Computing the rotation matrix from child link frame to world.
            relJointPos=np.matmul(relJointPos,frameToWorld) #jointPos relative to parentLink (in world coordinates)
            self.jointFrames[i][0]=np.asarray(linkState[0])+np.asarray(relJointPos) #world jointPos
            self.jointFrames[i][1]=self.multiply_quaternions(np.asarray(p.getJointInfo(self.world.ppsId,i)[15]),
                                                             np.asarray(p.getLinkState(self.world.ppsId,i-1)[1]))
            #joint quaternion = relative joint quaternion * child link quaternion
            localJointAxis=p.getJointInfo(self.world.ppsId,i)[13] #jointAxis (in joint frame)
            self.jointAxis[i-1]=np.matmul(np.asarray(localJointAxis),frameToWorld) #jointAxis (in world frame)
        return

    def update_jacobians(self):
        """Permet de récolter la matrice jacobienne de translation par rapport à chacun des points
        Cette fonction doit être vérifiée !"""
        velVec = [0 for i in range(0, 3)]
        accVec = velVec
        self.update_joint_pos(self.world.ppsId) #On update la position des joints histoire d'être sur
        self.update_joint_frame()
        for index in range(1, p.getNumJoints(self.world.ppsId)): #On parcourt 3 éléments
            localPoint = self.CoM_to_point(index, self.arrayRepLeoni[index-1][0])
            localCoM = self.frame_to_CoM(index)
            tempJac = p.calculateJacobian(self.world.ppsId, index, localPoint, self.jointPos, velVec, accVec)[0]
            self.arrayJacRep[index-1] = tempJac
            tempJac = p.calculateJacobian(self.world.ppsId, index, localCoM, self.jointPos, velVec, accVec)[0] #Always at CoM
            self.arrayJacAtt[index-1] = tempJac
            #logging.info("Test localInertialFramePosition: " + str(self.frame_to_CoM(index+1)))
        return

    def get_world_rep_forces(self):
        """Calcul des forces de répulsion dans l'espace 3D"""
        for i in range(0, len(self.arrayRepLeoni)):
            dist = self.arrayRepLeoni[i][2]
            dist = max(dist, 0.001) #avoid division by 0
            if dist <= self.rho0:
                vec = self.arrayRepLeoni[i][0] - self.arrayRepLeoni[i][1]
                grad = vec/np.linalg.norm(vec, 2) #computation of the gradient with the 2-norm
                self.arrayRepForces.append(self.eta*(1/dist-1/self.rho0)/(math.pow(dist, 2))*grad)
            else:
                self.arrayRepForces.append(np.zeros(3))
        return

    def get_world_att_forces(self):
        """Calcul des forces d'attraction dans l'espace 3D"""
        for i in range(0, len(self.arrayAttLeoni)):
            vec = self.arrayAttLeoni[i]-self.qFinal[i]
            dist = np.linalg.norm(vec, 2)
            if dist <= self.d:
                self.arrayAttForces.append(-self.zeta[i]*vec)
            else:
                self.arrayAttForces.append(-self.d*self.zeta[i]*vec/dist)
        return

    def proj_world_forces(self):
        """Projection des world forces sur les joints du robot et somme des contributions attractives et répulsives"""
        self.jointTorques=np.zeros(3) #Important de remettre à 0 le vecteur des torques
        for i in range(0, self.dofLeoni):
            projForce = np.matmul(np.asarray(self.arrayAttForces[i]), np.asarray(self.arrayJacAtt[i]))
            self.jointTorques += projForce
        for i in range(0, self.dofLeoni):
            projForce = np.matmul(np.asarray(self.arrayRepForces[i]), np.asarray(self.arrayJacRep[i]))
            self.jointTorques += projForce
        return

    def step_forward(self):
        """Effectue le step de position sur les joints et refresh la position
        avec la méthode du gradient descent"""
        normTorque = np.linalg.norm(self.jointTorques, 2)
        self.update_joint_pos(self.world.ppsId)
        nextPos=np.zeros(3)
        rho=np.linalg.norm(self.goal-self.jointPos,2)
        #alpha=0.001
        alpha=self.alpha0+(self.alphai-self.alpha0)/(np.linalg.norm(self.goal,2))*rho
        if not self.goalReached:
            for index in range(0,self.dofLeoni):
                """ATTENTION, IL FAUT CHANGER le vecteur GOAL si jamais !"""
                nextPos[index]= self.jointPos[index] + alpha*self.jointTorques[index]/normTorque
                p.setJointMotorControl2(self.world.ppsId, index+1, p.POSITION_CONTROL, targetPosition=nextPos[index],
                                        force=MAX_TORQUE,maxVelocity=MAX_SPEED)
        self.update_joint_pos(self.world.ppsId)
        return

    def check_accuracy(self):
        dist1 = np.linalg.norm(self.arrayAttLeoni[0] - self.qFinal[0], 2)
        dist2 = np.linalg.norm(self.arrayAttLeoni[1] - self.qFinal[1], 2)
        dist3=np.linalg.norm(self.arrayAttLeoni[2] - self.qFinal[2],2)
        if dist1<self.accuracy and dist2<self.accuracy and dist3<self.accuracy:
            self.goalReached=True
            self.alpha0=0.00001
            self.alphai=0.00001
        else:
            self.goalReached=False
        return

    def get_moving_joints(self, indexBody):
        """Array qui contient l'indice des joints mobiles"""
        movingJoints=[]
        for joint in range(0, p.getNumJoints(indexBody)):
            if p.getJointInfo(indexBody, joint)[2] != p.JOINT_FIXED: #Si le joint n'est pas fixe
                movingJoints.append(joint)
        return movingJoints

    def get_dof(self, indexBody):
        """Renvoie le nombre de dofs"""
        return len(self.movingJoints)

    def get_final_state(self):
        """Permet de récupérer les coordonnées des centres de masse finaux"""
        self.collisionThread.go_to_target_pos(self.goal)
        time.sleep(1.0) #Je veux laisser le temps pour qu'il atteigne la position en question
        for i in range(0, len(self.goal)):
            linkState = p.getLinkState(self.world.ppsId, i+1)
            self.qFinal.append(np.asarray(linkState[0]))

        #self.collisionThread.go_to_target_pos([0, 0, 0])
        return

    def multiply_quaternions(self,Q1,Q2):
        Q3=np.zeros(4)
        Q3[0]=Q1[3] * Q2[0] + Q1[0] * Q2[3] + Q1[1] * Q2[2] - Q1[2] * Q2[1]
        Q3[1]=Q1[3] * Q2[1] - Q1[0] * Q2[2] + Q1[1] * Q2[3] + Q1[2] * Q2[0]
        Q3[2]=Q1[3] * Q2[2] + Q1[0] * Q2[1] - Q1[1] * Q2[0] + Q1[2] * Q2[3]
        Q3[3]=Q1[3] * Q2[3] - Q1[0] * Q2[0] - Q1[1] * Q2[1] - Q1[2] * Q2[2]
        return Q3

    # -------------------------------------- PRINTERS --------------------------------------------------- #
    def print_array_rep_leoni(self):
        print("Repulsive points for Leoni:")
        for i in range(0,self.dofLeoni):
            print(self.arrayRepLeoni[i])
        return

    def print_array_att_leoni(self):
        print("Attractive points for Leoni")
        for i in range(0,self.dofLeoni):
            print(self.arrayAttLeoni[i])
        return

    def print_array_jac_rep(self):
        logging.info("Repulsive jacobian matrices")
        for i in range(0, self.dofLeoni):
            logging.info(str(self.arrayJacRep[i]))
        return

    def print_array_jac_att(self):
        logging.info("Attractive jacobian matrices")
        for i in range(0,self.dofLeoni):
            logging.info(str(self.arrayJacAtt[i]))
        return

    def print_att_forces(self):
        print("World attractive forces")
        for i in range(0,self.dofLeoni):
            print(str(self.arrayAttForces[i]))
        return

    def print_rep_forces(self):
        print("World repulsive forces")
        for i in range(0,self.dofLeoni):
            print(str(self.arrayRepForces[i]))
        return


    def print_joint_torques(self):
        logging.info("Joint torques to apply")
        for i in range(0, self.dofLeoni):
            logging.info(str(self.jointTorques[i]))
        return

    def print_joint_pos_deg(self):
        logging.info("Joint Position in degrees")
        for i in range(0, self.dofLeoni):
            logging.info(str(self.jointPos[i]*180/3.1415))
        return

    def print_position_all_link(self,indexBody):
        logging.info("Position de chacun des links")
        for link in range(0, p.getNumJoints(self.world.ppsId)):
            print(p.getLinkState(indexBody, link)[0])
        return
    # ---------------------------  Création du C-space -------------------------------------#

    def get_rep_fields(self):
        fields = []
        for i in range(0, len(self.arrayRepLeoni)):
            dist = self.arrayRepLeoni[i][2]
            dist = max(dist, 0.001)  #Distance inférieure au mm
            if dist <= self.rho0:
                fields.append(0.5 * math.pow((1 / dist) - 1 / self.rho0, 2))
            else:
                fields.append(0)
        self.arrayRepFields.append(np.asarray(fields))
        return

    def get_att_fields(self):
        fields = []
        for i in range(0, len(self.arrayAttLeoni)):
            vec = self.arrayAttLeoni[i] - self.qFinal[i]
            dist = np.linalg.norm(vec, 2)
            if dist <= self.d:
                fields.append(0.5 * self.zeta[i] * math.pow(dist, 2))
            else:
                fields.append(self.d * self.zeta[i] * dist - 0.5 * self.zeta[i] * math.pow(self.d, 2))
        self.arrayAttFields.append(np.asarray(fields))
        return

    def go_to_next(self):
        for i in range(0, self.dofLeoni - 1):
            p.setJointMotorControl2(self.world.ppsId, i + 1, p.POSITION_CONTROL, targetPosition=self.vectorPos[-1][i])
            #p.resetJointState(self.world.ppsId, i + 1, targetValue=self.vectorPos[-1][i])
            time.sleep(0.002)
            if self.collision:
                time.sleep(0.01)
        self.update_joint_pos(self.world.ppsId)
        return

    def append_vector_pos(self):
        tempCollision=self.collision
        if self.arrayRepObs[0][2]<0.0001:
            self.collision=True
        else:
            self.collision=False
        #print("Collision: " + str(self.collision))
        #logging.info("Temp Collision: " + str(tempCollision))
        #print("Signe: " + str(self.sign))
        if self.collision and not tempCollision: #detect a transition from true to false
            self.sign= -self.sign
            newPos = np.asarray([self.vectorPos[-1][0]+1*3.1415/180,self.vectorPos[-1][1]+self.sign*2*3.1415/180])
            self.posBlocage =self.vectorPos[-1][1]
            self.vectorPos.append(newPos)
            self.tour=0
        elif np.linalg.norm(self.vectorPos[-1][1]-self.posBlocage) >= 6.283*self.tour:
            newPos = np.asarray([self.vectorPos[-1][0]+1*3.1415/180,self.vectorPos[-1][1]+self.sign*1*3.1415/180])
            self.vectorPos.append(newPos)
            self.tour += 1
        else:
            newPos = np.asarray([self.vectorPos[-1][0], self.vectorPos[-1][1] + self.sign * 1* 3.1415 / 180])
            self.vectorPos.append(newPos)
        return

    def go_to_first_collision(self):
        p.resetJointState(self.world.ppsId, 2, targetValue=98 * 3.1415 / 180)
        time.sleep(1)
        p.resetJointState(self.world.ppsId, 1, targetValue=-180*3.1415/180)
        time.sleep(5)
        # p.resetJointState(self.world.ppsId, 2, targetValue=-30 * 3.1415 / 180)
        # time.sleep(1)
        # p.resetJointState(self.world.ppsId, 1, targetValue=0 * 3.1415 / 180)
        # time.sleep(5)
        self.update_joint_pos(self.world.ppsId)
        self.vectorPos.append(np.asarray(self.jointPos))
        self.update_arrays_rep()
        self.update_arrays_att()
        self.get_rep_fields()
        self.get_att_fields()
        return

    def write_Cspace(self,row):
        if not row:
            titles=['q1','q2','Uatt body1', 'Uatt body2','Uatt body3','Urep body1','Urep body2','Urep body3']
            for col in range(0,len(titles)):
                self.sheetExcelCspace.write(row,col,titles[col])
        else:
            csvData=[self.jointPos[0], self.jointPos[1], self.arrayAttFields[-1][0], self.arrayAttFields[-1][1],
             self.arrayAttFields[-1][2], self.arrayRepFields[-1][0], self.arrayRepFields[-1][1],
             self.arrayRepFields[-1][2]]
            for i in range(0,len(csvData)):
                self.sheetExcelCspace.write(row, i, csvData[i])
        return

    def write_forces(self,row):
        if not row:
            titles=['Fatt1x','Fatt1y','Fatt1z','Fatt2x','Fatt2y','Fatt2z','Fatt3x','Fatt3y','Fatt3z','Frep1x','Frep1y','Frep1z',
                    'Frep2x','Frep2y','Frep2z','Frep3x','Frep3y','Frep3z']
            for col in range(0,len(titles)):
                self.sheetExcelForces.write(row,col,titles[col])
        else:
            csvData=[self.arrayAttForces[0][0],self.arrayAttForces[0][1],self.arrayAttForces[0][2],
                     self.arrayAttForces[1][0],self.arrayAttForces[1][1],self.arrayAttForces[1][2],
                     self.arrayAttForces[2][0],self.arrayAttForces[2][1],self.arrayAttForces[2][2],
                     self.arrayRepForces[0][0], self.arrayRepForces[0][1], self.arrayRepForces[0][2],
                     self.arrayRepForces[1][0], self.arrayRepForces[1][1], self.arrayRepForces[1][2],
                     self.arrayRepForces[2][0],self.arrayRepForces[2][1],self.arrayRepForces[2][2]]
            for i in range(0,len(csvData)):
                self.sheetExcelForces.write(row, i, csvData[i])
        return

    def write_QST(self,row):
        if not row:
            titles=['q1','q2','q3','v1','v2','v3','T1','T2','T3']
            for col in range(0,len(titles)):
                self.sheetExcelQST.write(row,col,titles[col])
        else:
            jState=p.getJointStates(self.world.ppsId,[1,2,3])
            csvData=[jState[0][0],jState[1][0],jState[2][0],
                     jState[0][1],jState[1][1],jState[2][1],jState[0][3],jState[1][3],jState[2][3]]
            for i in range(0, len(csvData)):
                self.sheetExcelQST.write(row, i, csvData[i])
        return

    def write_CoMInfo(self,row):
        if not row:
            titles=['A1x','A1y','A1z','A2x','A2y','A2z','A3x','A3y','A3z','vx','vy','vz']
            for col in range(0,len(titles)):
                self.sheetExcelCoMInfo.write(row,col,titles[col])
        else:
            velocities=p.getLinkState(self.world.ppsId,2,1)[6] #cartesian velocity of A3 CoM
            csvData=[self.arrayAttLeoni[0][0],self.arrayAttLeoni[0][1],self.arrayAttLeoni[0][2],
                     self.arrayAttLeoni[1][0], self.arrayAttLeoni[1][1], self.arrayAttLeoni[1][2],
                    self.arrayAttLeoni[2][0],self.arrayAttLeoni[2][1],self.arrayAttLeoni[2][2],
                     velocities[0],velocities[1],velocities[2]]
            for i in range(0, len(csvData)):
                self.sheetExcelCoMInfo.write(row, i, csvData[i])
        return

    def construct_C_space(self):
        row=0
        self.write_Cspace(row)
        row +=1
        self.go_to_first_collision()
        while(self.jointPos[0]<160*3.1415/180):
            self.update_arrays_att()
            #self.print_array_att_leoni()
            self.update_arrays_rep()
            #self.print_array_rep_leoni()
            self.append_vector_pos()
            self.get_att_fields()
            self.get_rep_fields()
            self.write_Cspace(row)
            self.go_to_next()
            self.reinit_arrays_rep()
            row +=1
            time.sleep(0.001)
            #self.print_joint_pos_deg()

        return
