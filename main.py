from collisionThread import *
from FieldPlanner import *
from BulletWorld import *
from collisionThread import *
import logging
import sys
from typing import List
import atexit
import register

import xlsxwriter
# ----------------------------------------------------------------------------------
# GLOBALS TO DEFINE
# ----------------------------------------------------------------------------------


def __exit(fieldPlanner: FieldPlanner)-> None: #NE PAS mettre de return lorqu'on met "None"
    print("Program is about to close")
    #fieldPlanner.workbook.close()
    fieldPlanner.workbookT.close()
    sys.exit()


if __name__ == "__main__":
    goal= np.asarray([0.04,0.6542,0,0,0,0.8767])
    threshold_detection = 2
    logging.basicConfig(level=logging.DEBUG)
    logging.info("main function start")
    world = BulletWorld()
    collision = CollisionThread(world, threshold_detection, name='collisionTool')
    collision.start()  # Starts the thread
    fieldPlanner = FieldPlanner(goal, world, collision)
    fieldPlanner.start()
    atexit.register(__exit, fieldPlanner)
