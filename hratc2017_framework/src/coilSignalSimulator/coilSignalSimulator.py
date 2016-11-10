#!/usr/bin/python

from ConfigParser import ConfigParser
from minemapgenerator import MineMapGenerator, GenerateUsingRealDataset
from numpy import arange
from std_msgs.msg import Bool
from metal_detector_msgs.msg._Coil import Coil
import random, tf, rospy, rospkg

defaultpath = rospkg.RosPack().get_path("hratc2017_framework") + "/src/config/"
print defaultpath

class CoilSimulator(object):

    canStart = False

    def __init__(self):
	rospy.Subscriber("/configFirst",Bool,self.checkStart)
        r = rospy.Rate(10)
	#while self.canStart == False:
	#    r.sleep()
        self.load()
        self.listener = tf.TransformListener()
        self.pubMineDetection = rospy.Publisher("/coils", Coil)
        self.pubStartEveryone = rospy.Publisher("/configDone", Bool)

    def checkStart(self,data):
	if data.data == True:
	    self.canStart = True

    def load(self,path=defaultpath+"config.ini"):
        configFile = ConfigParser()
        configFile.read(path)
        self.mapWidth = configFile.getfloat("MapDimensions","width")
        self.mapHeight = configFile.getfloat("MapDimensions","height")
        self.resolution = configFile.getfloat("MapDimensions","resolution")

        self.numMines = configFile.getint("Mines","numMines")
        self.randomMines = configFile.getboolean("Mines","RandomMines")

        self.minDistDetection = configFile.getfloat("Mines","DetectionMinDist")
        self.maxDistExplosion = configFile.getfloat("Mines","ExplosionMaxDist")

        self.numCellsX  = self.mapWidth/self.resolution
        self.numCellsY  = self.mapHeight/self.resolution
        self.numCells   = self.numCellsX * self.numCellsY

        self.minesFixedPos = configFile.get("Mines","MinesPositions")
        self.generateMines()
        #self.save(path)

    def save(self,path):
        cfile = open(path,"w")

        configFile = ConfigParser()

        configFile.add_section("MapDimensions")
        configFile.set("MapDimensions","width",self.mapWidth)
        configFile.set("MapDimensions","height",self.mapHeight)
        configFile.set("MapDimensions","resolution",self.resolution)

        configFile.add_section("Mines")
        configFile.set("Mines","numMines",self.numMines)
        if(self.randomMines):
            configFile.set("Mines","RandomMines","true")
        else:
            configFile.set("Mines","RandomMines","false")
        self.minesFixedPos = ""
        if not self.randomMines:
            self.minesFixedPos = "|".join( ",".join(str(v) for v in r) for r in self.mines)

        configFile.set("Mines","MinesPositions",self.minesFixedPos)

        configFile.set("Mines","DetectionMinDist",self.minDistDetection)
        configFile.set("Mines","ExplosionMaxDist",self.maxDistExplosion)
        configFile.write(cfile)

        cfile.close()


    def generateMines(self):
        minesPos = self.minesFixedPos
        if minesPos !=  "":
            self.mines = [[float(v) for v in r.split(",")] for r in minesPos.split("|")]
        else:
            self.mines = []

        for i in arange(self.numMines-len(self.mines)):
            self.mines.append([ random.uniform(0.0,self.mapWidth)  - self.mapWidth/2., random.uniform(0.0,self.mapHeight) - self.mapHeight/2.])

        mWidth, mHeight = int(self.mapWidth/self.resolution),int(self.mapHeight/self.resolution)
        mines = [ [mWidth/2. + m[0]/self.resolution, mHeight/2. - m[1]/self.resolution] for m in self.mines]
	print "SELF:",self.mines
	print "MINES:",mines

        metals1 = []
        for i in arange(0.25*self.numMines):
            metals1.append([ random.uniform(0.0,self.mapWidth)  - self.mapWidth/2., random.uniform(0.0,self.mapHeight) - self.mapHeight/2.])

        metals2 = []
        for i in arange(0.25*self.numMines):
            metals2.append([
                                random.uniform(0.0,self.mapWidth)  - self.mapWidth/2.,
                                random.uniform(0.0,self.mapHeight) - self.mapHeight/2.])

        metals1 = [ [mWidth/2. + m[0]/self.resolution, mHeight/2. - m[1]/self.resolution] for m in metals1]
        metals2 = [ [mWidth/2. + m[0]/self.resolution, mHeight/2. - m[1]/self.resolution] for m in metals2]

        self.mineMap, self.zeroChannel = GenerateUsingRealDataset(mines,metals1,metals2,mWidth,mHeight,True)

        self.mines = [ [m[0]*self.resolution - self.mapWidth/2., -m[1]*self.resolution + self.mapHeight/2.] for m in mines]
        self.numMines = len(self.mines)

    def pubCoilsonMinefield(self):
        if self.listener == None:
            return
	
        # Lookup for the coils using tf
        try:
            (trans,rot) = self.listener.lookupTransform('minefield', 'left_coil', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return
        leftCoilX = trans[0]/self.resolution + self.numCellsX/2.0
        leftCoilY = -trans[1]/self.resolution + self.numCellsY/2.0

	print leftCoilX, leftCoilY
	
        try:
            (trans,rot) = self.listener.lookupTransform('minefield', 'middle_coil', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
          return
        middleCoilX = trans[0]/self.resolution + self.numCellsX/2.0
        middleCoilY = -trans[1]/self.resolution + self.numCellsY/2.0
	
        try:
            (trans,rot) = self.listener.lookupTransform('minefield', 'right_coil', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return
        rightCoilX = trans[0]/self.resolution + self.numCellsX/2.0
        rightCoilY = -trans[1]/self.resolution + self.numCellsY/2.0

        coils = [[leftCoilX,leftCoilY]]
        coils.append([middleCoilX, middleCoilY])
        coils.append([rightCoilX, rightCoilY])

	print self.mineMap.shape

        co = 0
        for x,y in coils:
            coil = Coil()
            coil.header.frame_id = "{}_coil".format(["left","middle","right"][co])
            if x <= self.mineMap.shape[2] and x >=0 and y <= self.mineMap.shape[1] and y >= 0:
                for ch in range(3):
                    coil.channel.append(self.mineMap[3*co+ch,y,x] + random.random()*100)
                    coil.zero.append(self.zeroChannel[3*co+ch])
		
                self.pubMineDetection.publish(coil)
            
            co += 1

if __name__=='__main__':
    print "CHEGOU\n\n"
    rospy.init_node('coilSignalSimulator')
    coilSimulator = CoilSimulator()

    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        coilSimulator.pubStartEveryone.publish(True)
        coilSimulator.pubCoilsonMinefield()
        r.sleep()



