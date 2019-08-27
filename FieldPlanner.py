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
from pytictoc import TicToc

import utils_collision
import register
logging.basicConfig(level=logging.DEBUG)

MAX_SPEED = 6*3.1415/180 #Medical speed
MAX_TORQUE = 0.96*171*9.2 #efficiency*gear ratio * motor torque
MAX_TCP_SPEED= 0.1
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
        self.eta=0.1
        # self.zeta=2000
        self.zeta = 15
        self.rho0=0.11
        self.d=2
        self.alphai=0.01
        self.alphat=0.0008
        self.D=1
        self.posBlocage=0
        self.tour=0
        self.sign=1
        self.collision=False
        self.arrayAttLeoni: List[ndarray] = []
        self.arrayAttObs: List[ndarray] = []
        basisList=[[0, 0, 0], [0, 0, 0], 100, "Name"]
        self.arrayRepLeoni = [basisList[:] for i in range(p.getNumJoints(self.world.ppsId)-1)] # Il est nécessaire d'initialiser la liste
        self.arrayRepObs = [basisList[:] for i in range(p.getNumJoints(self.world.myObstacle))]
        emptyJac = [[0 for i in range(3)] for y in range(3)]
        self.arrayJacAtt = emptyJac
        self.arrayJacAttRot = emptyJac
        self.arrayJacRep = [emptyJac[:] for i in range(p.getNumJoints(self.world.ppsId)-1)]
        self.jointPos = [0 for i in range(0, self.dofLeoni)]
        self.jointSpeed = [0 for i in range(0, self.dofLeoni)]
        self.jointFrames=[[[0 for i in range(3)],[0 for i in range(4)]]for i in range(p.getNumJoints(self.world.ppsId))]
        self.arrayRepFields: List[ndarray] = []
        self.arrayAttFields: List[ndarray] = []
        self.vectorPos: List[ndarray] = []
        self.arrayRepForces: List[ndarray] = []
        self.arrayAttForces: List[ndarray] = []
        self.jointAxis=[[0 for i in range(3)] for i in range(p.getNumJoints(self.world.ppsId))]
        self.jointTorques= np.zeros(3)
        self.goalReached=False
        self.cartAccuracy = 0.00008
        self.angAccuracy = 0.01*3.1415/180
        self.scalingTCP = 1
        self.myPositioningTime=TicToc()
        self.jacWrite=[[0 for i in range(6)] for y in range(3)]
        self.maxJointSpeed=0

        self.excelJointPos =xlsxwriter.Workbook('data_memoire6dofs/joint_positions_final.xlsx')
        self.sheetExcelJointPos= self.excelJointPos.add_worksheet()
        self.excelJointSpeed =xlsxwriter.Workbook('data_memoire6dofs/joint_velocities.xlsx')
        self.sheetExcelJointSpeed= self.excelJointSpeed.add_worksheet()
        self.excelJointTorques = xlsxwriter.Workbook('data_memoire6dofs/joint_torques.xlsx')
        self.sheetExcelJointTorques = self.excelJointTorques.add_worksheet()
        self.excelTCPInfo = xlsxwriter.Workbook('data_memoire6dofs/TCP_info_finalBis.xlsx')
        self.sheetExcelTCPInfo = self.excelTCPInfo.add_worksheet()
        self.excelForces = xlsxwriter.Workbook('data_memoire6dofs/forces.xlsx')
        self.sheetExcelForces = self.excelForces.add_worksheet()
        self.excelJacobians = xlsxwriter.Workbook('data_memoire6dofs/jacobians.xlsx')
        self.sheetExcelJacobians = self.excelJacobians.add_worksheet()

        self.localTCP=np.asarray([1.36,0,1.25]) #position du TCP par rapport au dernier joint (coordonnées locales)
        self.qFinal: List[ndarray] = self.qFinal_TCP_Iso()
        # self.jointsForXYZ = [1,1,0,0,0,1,0]
        self.jointsForRPY = [0,0,0,1,1,1,0]
        self.jointsForXYZ = [1,1,0,0,0,1,0]
        #self.jointsForRPY = [1,1,1,1,1,1,0]
        self.state="GoToXYZ"
        return

    def run(self):
        # Le but de ce thread va être de calculer la séquence de position que doit effectuer nos objets
        time.sleep(1)  # On attends une petite seconde le temps que l'autre thread commence à run
        row=0
        #self.get_final_state()
        row +=1
        substep = 0
        self.myPositioningTime.tic()
        #self.write_joint_pos(substep)
        #self.write_joint_speed(substep)
        #self.write_TCPInfo(substep)
        #self.write_joint_speed(substep)
        #self.write_torques(substep)
        self.write_forces(substep)
        #self.write_joint_pos(substep)
        while p.isConnected():
            self.update_arrays_att()
            self.update_arrays_rep()
            #self.show_joint()
            #self.show_array_att_Leoni()
            # # #self.get_att_fields()
            # # #self.get_rep_fields()
            self.update_jacobians()
            self.get_world_rep_forces()
            self.get_world_att_forces()
            self.proj_world_forces()
            # self.update_jacobian_att()
            # self.update_jacobian_rep()
            self.step_forward()
            if row // 5 > substep:
                substep +=1
                #self.write_joint_pos(substep)
                #self.write_joint_speed(substep)
                self.write_forces(substep)
                #self.write_torques(substep)
                #self.write_TCPInfo(substep)
            self.check_accuracy()
            self.reinit_arrays_rep() #Réinitialisation de la distance
            self.clear_arrays_forces()
            row +=1
        return

    def update_arrays_att(self):
        """Permet de récupérer les control points pour l'attraction vers le goal depuis collisionThread
        J'ai quand même gardé la matrice complète des centres de masse parce que c'est une info intéressante
        quand même"""
        self.clear_arrays_att() #On clear les listes
        self.update_joint_frame()
        for i in range(1, p.getNumJoints(self.world.ppsId)): #On commence à 1 pour éviter la base
            if i==p.getNumJoints(self.world.ppsId)-1: #Si on est sur la couch
                frameToWorld = np.reshape(np.asarray(p.getMatrixFromQuaternion(p.invertTransform(self.jointFrames[-1][0],
                                                                                                 self.jointFrames[-1][1])[1])),(3, 3))
                # On choppe la matrice de passage vers le world
                relglobalTCP = np.matmul(self.localTCP, frameToWorld)  #projection du vecteur TCP local sur le world
                self.arrayAttLeoni.append(self.jointFrames[-1][0]+relglobalTCP)  #On sélectionne la position absolue du centre de masse du link
            else:
                linkState = p.getLinkState(self.world.ppsId, linkIndex=i)
                self.arrayAttLeoni.append(np.asarray(linkState[0]))  # On sélectionne la position absolue du centre de masse du link
        for i in range(0, p.getNumJoints(self.world.myObstacle)):
            linkState = p.getLinkState(self.world.myObstacle, linkIndex=i)
            self.arrayAttObs.append(linkState[0])

        return

    def update_arrays_rep(self):
        """Permet de récupérer les control points pour l'esquive d'obstacle depuis collisionThread"""
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
            #print("Point on couch: " + str(self.arrayRepLeoni[6][2]))
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
        """localPoint est un vecteur allant du CoM au point de répulsion"""
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
        if indexBody==p.getNumJoints(self.world.ppsId)-1:
            localPoint= np.asarray(self.localTCP) #coordonnées locales du TCP directement
        else:
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

    def update_joint_speed(self,indexBody):
        for joint in range (0,self.dofLeoni):
            self.jointSpeed[joint] = p.getJointState(indexBody, joint+1)[1] #+1 pour éviter la base
        return

    def update_joint_frame(self):
        """Ici faut s'accrocher, en gros parentFramePos et ParentFrameOrn sont définis par rapport à la frame
        inertielle du link précédent. C'est pour ça que j'utilise getlinkstate pour chopper la position/orientation
        globale des frames intertielles pour chacun des links."""
        self.jointFrames[0][0]=np.asarray(p.getJointInfo(self.world.ppsId,0)[14]) #Position du joint 1 par rapport à la base
        self.jointFrames[0][1]=np.asarray(p.getJointInfo(self.world.ppsId,1)[15]) #Orientatation du joint 1 par rapport à la base
        self.jointAxis[0]=np.asarray(p.getJointInfo(self.world.ppsId,0)[13])
        for i in range(1, p.getNumJoints(self.world.ppsId)):
            relJointPos=np.asarray(p.getJointInfo(self.world.ppsId,i)[14]) #Position relative par rapport à la frame inertielle (exprimé dans cette frame locale)
            linkState=p.getLinkState(self.world.ppsId,i-1) #State du link précédent
            frameToWorld=np.reshape(np.asarray(p.getMatrixFromQuaternion(p.invertTransform(linkState[0],linkState[1])[1])),(3,3)) #On choppe la matrice de passage vers le world
            relJointPos=np.matmul(relJointPos,frameToWorld) #projection de la position sur le world
            self.jointFrames[i][0]=np.asarray(linkState[0])+np.asarray(relJointPos)
            self.jointFrames[i][1]=self.multiply_quaternions(np.asarray(p.getJointInfo(self.world.ppsId,i)[15]),np.asarray(p.getLinkState(self.world.ppsId,i-1)[1]))
            localJointAxis=p.getJointInfo(self.world.ppsId,i)[13]
            self.jointAxis[i]=np.matmul(np.asarray(localJointAxis),frameToWorld)
        return

    def update_jacobians(self):
        """Permet de récolter la matrice jacobienne de translation par rapport à chacun des points
        Cette fonction doit être vérifiée !"""
        velVec = [0 for i in range(self.dofLeoni)]
        accVec = velVec
        self.update_joint_pos(self.world.ppsId) #Update of joint position
        self.update_joint_frame() #Update of joint frame position and orientations
        for index in range(1, p.getNumJoints(self.world.ppsId)): #On parcourt 3 éléments
            localPoint = self.CoM_to_point(index, self.arrayRepLeoni[index-1][0])
            jac = p.calculateJacobian(self.world.ppsId, index, localPoint, self.jointPos, velVec, accVec)
            transJac=jac[0]
            transJac=self.select_subJac_trans(transJac)
            self.arrayJacRep[index-1] = transJac
            if index == p.getNumJoints(self.world.ppsId)-1:
                localCoM = self.frame_to_CoM(index)
                jac = p.calculateJacobian(self.world.ppsId, index, localCoM, self.jointPos, velVec, accVec) #Always at CoM
                transJac = jac[0]
                self.jacWrite = np.asarray(jac[0])
                self.update_joint_speed(self.world.ppsId)  # update of joint speeds
                normTCPspeed=np.linalg.norm(transJac)*np.linalg.norm(self.jointSpeed)
                #print("Norme TCP speed" + str(normTCPspeed))
                rotJac = jac[1]
                normJac = np.linalg.norm(transJac)
                transJac=self.select_subJac_trans(transJac)
                rotJac = self.select_subJac_rot(rotJac)

                if normTCPspeed > MAX_TCP_SPEED:
                    self.maxJointSpeed=0.8*MAX_TCP_SPEED/normJac
                else:
                    self.maxJointSpeed=MAX_SPEED
                self.arrayJacAtt= transJac
                self.arrayJacAttRot= rotJac
            #logging.info("Test localInertialFramePosition: " + str(self.frame_to_CoM(index+1)))
        return

    def select_subJac_trans(self,myJac):
        newJac=np.ndarray(shape=(3,self.jointsForXYZ.count(1)))
        col=0
        for i,enable in enumerate(self.jointsForXYZ):
            if enable:
                newJac[0][col]=myJac[0][i]
                newJac[1][col] = myJac[1][i]
                newJac[2][col] = myJac[2][i]
                col +=1
        return newJac

    def select_subJac_rot(self,myJac):
        newJac=np.ndarray(shape=(3,self.jointsForRPY.count(1)))
        col=0
        for i,enable in enumerate(self.jointsForRPY):
            if enable:
                newJac[0][col]=myJac[0][i]
                newJac[1][col] = myJac[1][i]
                newJac[2][col] = myJac[2][i]
                col +=1
        return newJac

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
        """Calcul de la force d'attraction du TCP dans l'espace 3D"""
        vec = self.arrayAttLeoni[-1]-self.qFinal[0]
        dist = np.linalg.norm(vec, 2)
        if dist <= self.d:
            self.arrayAttForces.append(-self.zeta*vec)
        else:
            self.arrayAttForces.append(-self.d*self.zeta*vec/dist)
        return

    def proj_world_forces(self):
        """Projection des world forces sur les joints du robot et somme des contributions attractives et répulsives"""
        self.jointTorques=np.zeros(self.jointsForXYZ.count(1))
        #Important de remettre à 0 le vecteur des torques, la taille est égale au nombre de enable joints
        projForce = np.matmul(self.arrayAttForces[0], self.arrayJacAtt) #Pour le TCP
        self.jointTorques += projForce
        for i in range(0, len(self.arrayRepForces)):
            projForce = np.matmul(self.arrayRepForces[i], self.arrayJacRep[i])
            self.jointTorques += projForce
        return

    def step_forward(self):
        """Effectue le step de position sur les joints et refresh la position
        avec la méthode du gradient descent"""
        normTorque = np.linalg.norm(self.jointTorques, 2)
        self.update_joint_pos(self.world.ppsId)
        nextPos=np.zeros(self.dofLeoni)
        rho=np.linalg.norm(self.qFinal[0]-np.asarray(self.arrayAttLeoni[-1]),2)
        indexTT=0
        indexTR=0
        """ --------------------------------  Cartesian Part ----------------------------"""
        if not self.goalReached:
            for index,enable in enumerate(self.jointsForXYZ):
                if enable:
                    alpha = self.alphat + (self.alphai - self.alphat) / (np.linalg.norm(self.qFinal[0]-self.localTCP, 2)) * rho
                    nextPos[index] = self.jointPos[index] + alpha * self.D*self.jointTorques[indexTT] / normTorque
                    p.setJointMotorControl2(self.world.ppsId, index+1, p.POSITION_CONTROL, targetPosition=nextPos[index],
                                            force=MAX_TORQUE,maxVelocity=self.maxJointSpeed)
                    indexTT +=1

        """ --------------------------------  Orientation Part ----------------------------"""
        targetQuat=p.getQuaternionFromEuler(self.qFinal[1]) #orientation désirée du TCP
        TCPToWorld=p.invertTransform([0,0,0],p.getLinkState(self.world.ppsId,self.dofLeoni+1)[1])[1] #quaternion de transformation entre les 2 orientations
        TCPToTarget=self.multiply_quaternions(targetQuat,TCPToWorld) #Conversion en angles d'Euler
        TCPToTarget=p.getEulerFromQuaternion(TCPToTarget)
        errorAngles=np.matmul(np.array(TCPToTarget),self.arrayJacAttRot) #Projection des erreurs d'angles en erreur de coordonnées robots

        if not self.goalReached:
            for index, enable in enumerate(self.jointsForRPY):
                if enable:
                    nextPos[index] = self.jointPos[index]+errorAngles[indexTR]
                    p.setJointMotorControl2(self.world.ppsId, index + 1, p.POSITION_CONTROL,
                                            targetPosition=nextPos[index], force=MAX_TORQUE,
                                            maxVelocity=self.maxJointSpeed)
                    indexTR +=1
            self.update_joint_pos(self.world.ppsId)
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

    def multiply_quaternions(self,Q1,Q2):
        Q3=np.zeros(4)
        Q3[0]=Q1[3] * Q2[0] + Q1[0] * Q2[3] + Q1[1] * Q2[2] - Q1[2] * Q2[1]
        Q3[1]=Q1[3] * Q2[1] - Q1[0] * Q2[2] + Q1[1] * Q2[3] + Q1[2] * Q2[0]
        Q3[2]=Q1[3] * Q2[2] + Q1[0] * Q2[1] - Q1[1] * Q2[0] + Q1[2] * Q2[3]
        Q3[3]=Q1[3] * Q2[3] - Q1[0] * Q2[0] - Q1[1] * Q2[1] - Q1[2] * Q2[2]
        return Q3

    # def compute_Laplacian_Field(self):
    #     self.D=1 #reinitialization of D
    #     for i in range(0, len(self.arrayRepLeoni)):
    #         vec=self.arrayRepLeoni[i][0]-self.arrayRepLeoni[i][1]
    #         rho=self.arrayRepLeoni[i][2]
    #         grad=vec/rho
    #         gradT=np.reshape(grad,(3,1))
    #         laplacianRho=2/rho
    #         self.D += self.eta*1/math.pow(rho,4)*np.matmul(grad,gradT)
    #         self.D += 2*self.eta*(1/rho-1/self.rho0)/math.pow(rho,3)*np.matmul(grad,gradT)
    #         self.D -= self.eta*(1/rho-1/self.rho0)*1/math.pow(rho,2)*laplacianRho
    #     self.D +=3*self.zeta #attractive part
    #     self.D = 1/self.D
    #     return

    def qFinal_TCP_Iso(self) -> List[ndarray]:
        qFinal: List[ndarray]=[]
        qFinal.append(np.asarray([0,0,0]))
        #qFinal.append(np.asarray([-90*3.1415/180,10*3.1415/180,0]))
        qFinal.append(np.asarray([0, 0, 90*3.1415/180]))
        return qFinal

    def check_accuracy(self):
        triggerReached=self.goalReached
        dist=np.linalg.norm(self.qFinal[0]-self.arrayAttLeoni[-1],2)
        orientation=p.getEulerFromQuaternion(p.getLinkState(self.world.ppsId,self.dofLeoni+1)[1])
        angle=np.linalg.norm(self.qFinal[1]-np.asarray(orientation),2)

        if dist < self.cartAccuracy and angle < self.angAccuracy:
            self.goalReached = True
            self.alphai = 0
            self.alphat = 0
            if not triggerReached:
                print("Goal Reached")
                self.stop_timer()
        return

    def stop_timer(self):
        self.myPositioningTime.toc()
        print("Temps écoulé pour le placement du patient: " + str(self.myPositioningTime))
        return

    # -------------------------------------- PRINTERS --------------------------------------------------- #
    def print_array_rep_leoni(self):
        print("Repulsive points for Leoni:")
        for elem in self.arrayRepLeoni:
            print(elem)
        return

    def print_array_att_leoni(self):
        print("Attractive points for Leoni")
        for elem in self.arrayAttLeoni:
            print(elem)
        return

    def print_array_jac_rep(self):
        logging.info("Repulsive jacobian matrices")
        for elem in self.arrayJacRep:
            logging.info(str(elem))
        return

    def print_array_jac_att(self):
        logging.info("Attractive jacobian matrices")
        for elem in self.arrayJacAtt:
            logging.info(str(elem))
        return

    def print_att_forces(self):
        print("World attractive forces")
        for elem in self.arrayAttForces:
            print(str(elem))
        return

    def print_rep_forces(self):
        print("World repulsive forces")
        for elem in self.arrayRepForces:
            print(str(elem))
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

    def show_joint(self):
        self.update_joint_frame()
        for jointFrame in self.jointFrames:
            p.addUserDebugLine(lineFromXYZ=jointFrame[0], lineToXYZ=[jointFrame[0][0],jointFrame[0][1]-1,jointFrame[0][2]
                                                                    ], lineColorRGB=[1, 0, 0], lineWidth=4,lifeTime=1)
            p.addUserDebugLine(lineFromXYZ=jointFrame[0], lineToXYZ=[jointFrame[0][0],jointFrame[0][1],jointFrame[0][2]+1
                                                                    ], lineColorRGB=[1, 0, 0], lineWidth=4,lifeTime=1)
        return

    def show_array_att_Leoni(self):
        for i,attPoint in enumerate(self.arrayAttLeoni):
            if i==len(self.arrayAttLeoni)-1:
                p.addUserDebugLine(lineFromXYZ=attPoint, lineToXYZ=[attPoint[0],attPoint[1],attPoint[2]+1
                                                                        ], lineColorRGB=[0, 1, 0], lineWidth=4,lifeTime=1)
                p.addUserDebugLine(lineFromXYZ=attPoint, lineToXYZ=[attPoint[0],attPoint[1]-1,attPoint[2]
                                                                        ], lineColorRGB=[0, 1, 0], lineWidth=4,lifeTime=1)
            else:
                p.addUserDebugLine(lineFromXYZ=attPoint, lineToXYZ=[attPoint[0], attPoint[1], attPoint[2] + 1
                                                                    ], lineColorRGB=[0, 0, 1], lineWidth=4,
                                   lifeTime=1)
                p.addUserDebugLine(lineFromXYZ=attPoint, lineToXYZ=[attPoint[0], attPoint[1] - 1, attPoint[2]
                                                                    ], lineColorRGB=[0, 0, 1], lineWidth=4,lifeTime=1)
        return

    def show_array_rep_Leoni(self):
        for i in range (len(self.arrayRepLeoni)):
            if self.arrayRepLeoni[i][2]!=100:
                my_line=p.addUserDebugLine(lineFromXYZ=self.arrayRepLeoni[i][0], lineToXYZ=self.arrayRepLeoni[i][1], lineColorRGB=[1, 1, 0], lineWidth=2,
                               lifeTime=1)
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
        vec = self.arrayAttLeoni[-1] - self.qFinal[0]
        dist = np.linalg.norm(vec, 2)
        if dist <= self.d:
            fields.append(0.5 * self.zeta * math.pow(dist, 2))
        else:
            fields.append(self.d * self.zeta * dist - 0.5 * self.zeta * math.pow(self.d, 2))
        self.arrayAttFields.append(np.asarray(fields))
        return

    def go_to_next(self):
        for i in range(0, self.dofLeoni - 1):
            p.setJointMotorControl2(self.world.ppsId, i + 1, p.POSITION_CONTROL, targetPosition=self.vectorPos[-1][i])
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
        p.resetJointState(self.world.ppsId, 2, targetValue=93 * 3.1415 / 180)
        time.sleep(1)
        p.resetJointState(self.world.ppsId, 1, targetValue=-180*3.1415/180)
        time.sleep(5)
        self.update_joint_pos(self.world.ppsId)
        self.vectorPos.append(np.asarray(self.jointPos))
        self.update_arrays_rep()
        self.update_arrays_att()
        self.get_rep_fields()
        self.get_att_fields()
        return

    def write_joint_pos(self,row):
        if not row:
            titles=['q1','q2','q3','q4','q5','q6']
            for col in range(0,len(titles)):
                self.sheetExcelJointPos.write(row,col,titles[col])
        else:
            self.update_joint_pos(self.world.ppsId)
            csvData=[self.jointPos[0], self.jointPos[1],self.jointPos[2],self.jointPos[3],self.jointPos[4],self.jointPos[5]]
            for i in range(0,len(csvData)):
                self.sheetExcelJointPos.write(row, i, csvData[i])
        return

    def write_joint_speed(self,row):
        if not row:
            titles=['dot q1','dot q2','dot q3','dot q4','dot q5','dot q6']
            for col in range(0,len(titles)):
                self.sheetExcelJointSpeed.write(row,col,titles[col])
        else:
            self.update_joint_speed(self.world.ppsId)
            csvData=[self.jointSpeed[0], self.jointSpeed[1],self.jointSpeed[2],self.jointSpeed[3],self.jointSpeed[4]
                ,self.jointSpeed[5]]
            for i in range(0,len(csvData)):
                self.sheetExcelJointSpeed.write(row, i, csvData[i])
        return

    def write_torques(self,row):
        if not row:
            titles=['T1','T2','T3','T4','T5','T6']
            for col in range(0,len(titles)):
                self.sheetExcelJointTorques.write(row,col,titles[col])
        else:
            jState=p.getJointStates(self.world.ppsId,self.movingJoints)
            csvData=[jState[0][3],jState[1][3],jState[2][3],
                     jState[3][3],jState[4][3],jState[5][3]]
            for i in range(0, len(csvData)):
                self.sheetExcelJointTorques.write(row, i, csvData[i])
        return

    def write_forces(self, row):
        if not row:
            titles = ['Fattx','Fatty','Fattz']
            for col in range(0, len(titles)):
                self.sheetExcelForces.write(row, col, titles[col])
        else:
            csvData = [self.arrayAttForces[-1][0],self.arrayAttForces[-1][1],self.arrayAttForces[-1][2]]
            for i in range(0, len(csvData)):
                self.sheetExcelForces.write(row, i, csvData[i])
        return

    def write_TCPInfo(self, row):
        if not row:
            titles = ['x','y','z','psi','theta','phi','vx','vy','vz']
            for col in range(0, len(titles)):
                self.sheetExcelTCPInfo.write(row, col, titles[col])
        else:
            self.update_joint_speed(self.world.ppsId)
            velTCP=np.zeros(3)
            velTCP=np.matmul(self.jacWrite,np.asarray(self.jointSpeed))
            orientation=p.getEulerFromQuaternion(p.getLinkState(self.world.ppsId,self.dofLeoni+1)[1])
            csvData = [self.arrayAttLeoni[-1][0],self.arrayAttLeoni[-1][1],self.arrayAttLeoni[-1][2],
                       orientation[0],orientation[1],orientation[2],velTCP[0],velTCP[1],velTCP[2]]
            for i in range(0, len(csvData)):
                self.sheetExcelTCPInfo.write(row, i, csvData[i])
        return

    def write_jacobians(self):
        line=0
        separation=['Jatt1','Jatt7','Jrep1','Jrep2','Jrep3','Jrep4','Jrep5','Jrep6','Jrep7']
        csvData=[separation[0]]
        self.sheetExcelJacobians.write(line, 0, csvData[0])
        print("My current Jacobian: " + str(csvData))
        line += 1
        for row in range(3):
            csvData=[self.arrayJacAtt[0],self.arrayJacAtt[1],self.arrayJacAtt[2],self.arrayJacAtt[3]
                    ,self.arrayJacAtt[4],self.arrayJacAtt[5]]
            print("My row in jacobian: " + str(csvData))
            for col in range(len(csvData)):
                self.sheetExcelJacobians.write(line, col, csvData[col])
            line +=1
        for jac in range(len(self.arrayJacRep)):
            csvData = [separation[jac+len(self.arrayJacAtt)]]
            self.sheetExcelJacobians.write(line, 0, csvData[0])
            print("My current Jacobian: " + str(csvData))
            line += 1
            currentJac = self.arrayJacRep[jac]
            for row in range(3):
                csvData = [currentJac[row][0], currentJac[row][1], currentJac[row][2], currentJac[row][3]
                    , currentJac[row][4], currentJac[row][5]]
                print("My row in jacobian: " + str(csvData))
                for col in range(len(csvData)):
                    self.sheetExcelJacobians.write(line, col, csvData[col])
                line += 1
        return
        
    def construct_C_space(self):
        row=0
        #self.write_csv(row)
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
            #self.write_csv(row)
            self.go_to_next()
            self.reinit_arrays_rep()
            row +=1
            time.sleep(0.001)
            #self.print_joint_pos_deg()

        return
