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


class FieldPlanner(threading.Thread):

    def __init__(self,
                 goal: List[float], world,
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
        self.eta = 1000000
        self.zeta = 10
        self.rho0 = 100
        self.d = 10
        self.alpha = 0.01
        self.posBlocage=0
        self.tour=0
        self.sign=-1
        self.collision=False
        self.arrayAttLeoni: List[ndarray] = []
        self.arrayAttObs: List[ndarray] = []
        basisList=[[0, 0, 0], [0, 0, 0], 10000, "Name"]
        self.arrayRepLeoni = [basisList[:] for i in range(p.getNumJoints(self.world.ppsId)-1)] # Il est n�cessaire d'initialiser la liste
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
        self.workbook = xlsxwriter.Workbook('trajectory_Obs_new.xlsx')
        self.worksheet = self.workbook.add_worksheet()

        """self.qFinal: List[ndarray] = [np.asarray([-42.134,-230.351,-110.3]),np.asarray([-91.93,-272.13,-74.3])
            ,np.asarray([-119.87,-212.19,-47.3])] #Pour le goal (40,75,0)"""
        self.qFinal: List[ndarray] = [np.asarray([0.865, -249.99, -110.3]), np.asarray([1.8875, -314.985, -74.3])
            , np.asarray([54.087, -355.585, -47.3])]  # Pour le goal (90.947,-128.842,0)
        return

    def run(self):
        # Le but de ce thread va �tre de calculer la s�quence de position que doit effectuer nos objets
        time.sleep(1)  # On attends une petite seconde le temps que l'autre thread commence � run
        #self.get_final_state()
        row=0
        #self.construct_C_space()
        while p.isConnected():
            #self.construct_C_space()
            self.update_arrays_att()
            #self.print_array_att_leoni()
            self.update_arrays_rep()
            self.get_att_fields()
            self.get_rep_fields()
            self.write_joint_pos(row)
            #self.print_array_rep_leoni()
            self.update_jacobians()
            #self.print_array_jac_att()
            #self.print_array_jac_rep()
            self.get_world_rep_forces()
            self.get_world_att_forces()
            #self.print_rep_forces()
            self.print_att_forces()
            self.proj_world_forces()
            self.step_forward()
            #self.print_joint_torques()
            #self.print_joint_torques(jointTorques)
            #self.print_joint_pos_deg()
            self.reinit_arrays_rep() #R�initialisation de la distance
            self.clear_arrays_forces()
            row +=1
            #self.print_position_all_link(self.world.ppsId)
        print("finito")

        return

    def update_arrays_att(self):
        """Permet de r�cup�rer les control points pour l'attraction vers le goal depuis collisionThread"""
        self.clear_arrays_att() #On clear les listes
        for i in range(1, p.getNumJoints(self.world.ppsId)): #On commence � 1 pour �viter la base
            linkState = p.getLinkState(self.world.ppsId, linkIndex=i)
            self.arrayAttLeoni.append(linkState[0]) #On s�lectionne la position absolue du centre de masse du link
        for i in range(0, p.getNumJoints(self.world.myObstacle)):
            linkState = p.getLinkState(self.world.myObstacle, linkIndex=i)
            self.arrayAttObs.append(linkState[0])

        return

    def update_arrays_rep(self):
        """Permet de r�cup�rer les control points pour l'esquive d'obstacle depuis collisionThread"""
        tempList = []
        listCouples = self.collisionThread.listCouples
        results = self.collisionThread.results
        for i in range(0, len(listCouples)):
            ind = listCouples[i][0][1]-1 #index permettant de se placer correctement dans l'arrayRepLeoni, -1
            # pour la base
            myJointInfo = p.getJointInfo(self.world.myObstacle, listCouples[i][1][1])
            if results[2][i] < self.arrayRepLeoni[ind][2]:
                self.arrayRepLeoni[ind][0] = np.asarray(results[1][i][0])
                self.arrayRepLeoni[ind][1] = np.asarray(results[1][i][1])
                self.arrayRepLeoni[ind][2] = results[2][i]
                self.arrayRepLeoni[ind][3] = myJointInfo[12] #link name
            ind = listCouples[i][1][1]
            myJointInfo = p.getJointInfo(self.world.ppsId, listCouples[i][0][1])
            if results[2][i] < self.arrayRepObs[ind][2]:
                self.arrayRepObs[ind][0] = np.asarray(results[1][i][1]) #Inversion des vecteurs
                self.arrayRepObs[ind][1] = np.asarray(results[1][i][0])
                self.arrayRepObs[ind][2] = results[2][i]
                self.arrayRepObs[ind][3] = myJointInfo[12]  # link name
        return

    def reinit_arrays_rep(self):
        for i in range(0, len(self.arrayRepLeoni)):
            self.arrayRepLeoni[i][2] = 10000 #La distance doit �tre r�initialis�e
        for i in range(0, len(self.arrayRepObs)):
            self.arrayRepObs[i][2] = 10000
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
        """localPoint est un vecteur allant du CoM au point de r�pulsion
        � v�rifier"""
        localPoint = tuple(np.subtract(worldPoint, self.jointFrames[indexBody][0])) #vecteur coordonn�es absolues � projeter
        linkState=p.getLinkState(self.world.ppsId,indexBody) #Ici j'avais mis -1 puis jl'ai retir� et ca a fonctionn�
        #Correspond � la transformation du world vers la frame locale.
        worldToFrame = p.getMatrixFromQuaternion(linkState[5])#On transforme le quaternion en matrice de rotation
        worldToFrame = np.asarray(worldToFrame)
        localPoint = np.asarray(localPoint) #vecteur entre CoM et point de r�pulsion � projeter dans la frame locale
        worldToFrame = np.reshape(worldToFrame, (3, 3))
        localPoint = tuple(np.matmul(localPoint,worldToFrame)) #projection dans la frame locale
        return localPoint

    def frame_to_CoM(self, indexBody) -> tuple:
        """Permet de r�cup�rer l'attribut localInertialFramePosition
        valeur xyz de "inertial" dans l'urdf"""
        localPoint = p.getLinkState(self.world.ppsId, indexBody)[2]
        return localPoint

    def update_joint_pos(self, indexBody):
        """"Met � jour la liste contenant la position des joints(sans la base)
        On a besoin des joints int�rm�diaires pour la matrice jacobienne dans tous les cas
        doit avoir la m�me taille que dof
        """
        for joint in range(0, self.dofLeoni):
            self.jointPos[joint] = p.getJointState(indexBody, joint+1)[0] #+1 pour �viter la base

        return

    def update_joint_frame(self):
        """Ici faut s'accrocher, en gros parentFramePos et ParentFrameOrn sont d�finis par rapport � la frame
        inertielle du link pr�c�dent. C'est pour �a que j'utilise getlinkstate pour chopper la position/orientation
        globale des frames intertielles pour chacun des links."""
        self.jointFrames[0][0]=np.asarray(p.getJointInfo(self.world.ppsId,0)[14])
        self.jointFrames[0][1]=np.asarray(p.getJointInfo(self.world.ppsId,1)[15])
        for i in range(1, p.getNumJoints(self.world.ppsId)):
            relJointPos=np.asarray(p.getJointInfo(self.world.ppsId,i)[14]) #Position relative par rapport � la frame inertielle (exprim� dans cette frame locale)
            linkState=p.getLinkState(self.world.ppsId,i-1) #State du link pr�c�dent
            frameToWorld=np.reshape(np.asarray(p.getMatrixFromQuaternion(p.invertTransform(linkState[0],linkState[1])[1])),(3,3)) #On choppe la matrice de passage vers le world
            relJointPos=np.matmul(relJointPos,frameToWorld) #projection de la position sur le world
            self.jointFrames[i][0]=np.asarray(linkState[0])+np.asarray(relJointPos)
            self.jointFrames[i][1]=self.multiply_quaternions(np.asarray(p.getJointInfo(self.world.ppsId,i)[15]),np.asarray(p.getLinkState(self.world.ppsId,i-1)[1]))
            localJointAxis=p.getJointInfo(self.world.ppsId,i)[13]
            self.jointAxis[i-1]=np.matmul(np.asarray(localJointAxis),frameToWorld)
        return

    def update_jacobians(self):
        """Permet de r�colter la matrice jacobienne de translation par rapport � chacun des points
        Cette fonction doit �tre v�rifi�e !"""
        velVec = [0 for i in range(0, 3)]
        accVec = velVec
        self.update_joint_pos(self.world.ppsId) #On update la position des joints histoire d'�tre sur
        self.update_joint_frame()
        for index in range(1, p.getNumJoints(self.world.ppsId)): #On parcourt 3 �l�ments
            localPoint = self.CoM_to_point(index, self.arrayRepLeoni[index-1][0])
            localCoM = self.frame_to_CoM(index)
            tempJac = p.calculateJacobian(self.world.ppsId, index, localPoint, self.jointPos, velVec, accVec)[0]
            self.arrayJacRep[index-1] = tempJac
            tempJac = p.calculateJacobian(self.world.ppsId, index, localCoM, self.jointPos, velVec, accVec)[0] #Always at CoM
            self.arrayJacAtt[index-1] = tempJac
            #logging.info("Test localInertialFramePosition: " + str(self.frame_to_CoM(index+1)))
        return

    def get_world_rep_forces(self):
        """Calcul des forces de r�pulsion dans l'espace 3D"""
        for i in range(0, len(self.arrayRepLeoni)):
            dist = self.arrayRepLeoni[i][2]
            dist = max(dist, 0.01) #avoid division by 0
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
                self.arrayAttForces.append(-self.zeta*vec)
            else:
                self.arrayAttForces.append(-self.d*self.zeta*vec/dist)
        return

    def proj_world_forces(self):
        """Projection des world forces sur les joints du robot et somme des contributions attractives et r�pulsives"""
        self.jointTorques=np.zeros(3) #Important de remettre � 0 le vecteur des torques
        for i in range(0, self.dofLeoni):
            projForce = np.matmul(np.asarray(self.arrayAttForces[i]), np.asarray(self.arrayJacAtt[i]))
            self.jointTorques += projForce
        for i in range(0, self.dofLeoni):
            projForce = np.matmul(np.asarray(self.arrayRepForces[i]), np.asarray(self.arrayJacRep[i]))
            self.jointTorques += projForce
        return

    def step_forward(self):
        """Effectue le step de position sur les joints et refresh la position
        avec la m�thode du gradient descent"""
        normTorque = np.linalg.norm(self.jointTorques, 2)
        for index in range(0,self.dofLeoni):
            nextPos= self.jointPos[index] + self.alpha*self.jointTorques[index]/normTorque
            p.setJointMotorControl2(self.world.ppsId, index+1, p.POSITION_CONTROL, targetPosition=nextPos)
            #p.resetJointState(self.world.ppsId, index+1, targetValue=nextPos)
        self.update_joint_pos(self.world.ppsId)
        #print("This is the vector of torques: " + str(jointTorques))
        #logging.info("Next Position in degrees:" + str([elem*180/3.1415 for elem in q]))
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
        """Permet de r�cup�rer les coordonn�es des centres de masse finaux"""
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
    # ---------------------------  Cr�ation du C-space -------------------------------------#

    def get_rep_fields(self):
        fields = []
        for i in range(0, len(self.arrayRepLeoni)):
            dist = self.arrayRepLeoni[i][2]
            dist = max(dist, 0.01)  # avoid division by 0
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
                fields.append(0.5 * self.zeta * math.pow(dist, 2))
            else:
                fields.append(self.d * self.zeta * dist - 0.5 * self.zeta * math.pow(self.d, 2))
        self.arrayAttFields.append(np.asarray(fields))
        return

    def go_to_next(self):
        for i in range(0, self.dofLeoni - 1):
            #print("This is my vectorPos:" + str(self.vectorPos[-1][i]*180/3.1415))
            p.setJointMotorControl2(self.world.ppsId, i + 1, p.POSITION_CONTROL, targetPosition=self.vectorPos[-1][i])
            #p.resetJointState(self.world.ppsId, i + 1, targetValue=self.vectorPos[-1][i])
            time.sleep(0.002)
            if self.collision:
                time.sleep(0.01)
        self.update_joint_pos(self.world.ppsId)
        return

    def append_vector_pos(self):
        tempCollision=self.collision
        self.update_joint_pos(self.world.ppsId)
        if self.arrayRepObs[0][2]<0.01:
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
        p.setJointMotorControl2(self.world.ppsId, 2, p.POSITION_CONTROL, targetPosition=93 * 3.1415 / 180)
        time.sleep(1)
        p.setJointMotorControl2(self.world.ppsId, 1, p.POSITION_CONTROL, targetPosition=-180 * 3.1415 / 180)
        time.sleep(5)
        self.update_joint_pos(self.world.ppsId)
        self.update_arrays_att()
        self.update_arrays_rep()
        self.get_att_fields()
        self.get_rep_fields()
        self.vectorPos.append(np.asarray(self.jointPos))
        return

    def write_csv(self, row):
        if not row:
            titles=['q1','q2','Uatt body1','Uatt body2','Uatt body3','Urep body1','Urep body2','Urep body3']
            for col in range(0,len(titles)):
                self.worksheet.write(row,col,titles[col])
        else:
            csvData=[self.vectorPos[-1][0], self.vectorPos[-1][1], self.arrayAttFields[-1][0], self.arrayAttFields[-1][1],
            self.arrayAttFields[-1][2], self.arrayRepFields[-1][0], self.arrayRepFields[-1][1],
             self.arrayRepFields[-1][2]]
            for i in range(0,len(csvData)):
                self.worksheet.write(row, i, csvData[i])
        return

    def write_joint_pos(self,row):
        if not row:
            titles=['q1','q2','q3','Uatt body1', 'Uatt body2','Uatt body3','Urep body1','Urep body2','Urep body3']
            for col in range(0,len(titles)):
                self.worksheet.write(row,col,titles[col])
        else:
            self.update_joint_pos(self.world.ppsId)
            csvData=[self.jointPos[0], self.jointPos[1], self.jointPos[2], self.arrayAttFields[-1][0], self.arrayAttFields[-1][1],
             self.arrayAttFields[-1][2], self.arrayRepFields[-1][0], self.arrayRepFields[-1][1],
             self.arrayRepFields[-1][2]]
            for i in range(0,len(csvData)):
                self.worksheet.write(row, i, csvData[i])
        return

    def construct_C_space(self):
        row=0
        self.write_csv(row)
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
            self.write_csv(row)
            self.go_to_next()
            self.reinit_arrays_rep()
            row +=1
            #self.print_joint_pos_deg()

        return