from agent import *
from math import *
import random
from heapq import *

class StudentAgent(Agent):
    
    def __init__(self, name, body, world):
        super().__init__(name, body, world)
        #config values
        self.lazysteps = 1000 #step limits exist in case something goes wrong
        self.normalsteps = 800
        self.mpanicsteps = 700
        self.spanicsteps = 600
        self.escapesteps = 1000 #step limit to escape rooms
        self.approachsteps = 5000 #step limit to move closer to the middle of the map
        self.maxapproach = 40 #max turns without finding food
        self.escaperange = 5 #new point to try to escape, must be 2 or higher
        self.mindist = 11 #escape if did not move more than this lately
        self.maxbored = 25 #amount of turns without changing direction
        self.boredrange = 2 #new point next to head, must be 2 or higher
        self.maxpastturns = 20 #how old the saved position is
        self.futuresteps = 50 #step limit for safeAction
        self.maxwait = 10 #max amount of turns spent waiting for the other agent
        self.maxcd = 25 #trade cooldown
        #other values
        self.lastact = None #saved action (no path saved)
        self.currentpath = [] #saved path, given by findPath()
        self.goal = None #current goal        
        self.bored = self.maxbored #uneventful turns before changing direction        
        self.danger = set() #unsafe position cache        
        self.past = (0,0) #where it was at least maxpastturns ago        
        self.pastturns = self.maxpastturns
        self.lastMvalue = 0
        self.lastSvalue = 0
        self.theirX = 0 #ally agent's head[0]
        self.theirY = 0 #ally agent's head[1]
        self.approach = self.maxapproach
        self.waittime = self.maxwait
        self.cd = 0
        self.avoid = (-1,-1) #consider this position as a wall
        #flags
        self.goingforfood = False #True = the path leads towards food
        self.trading = False #True = moving towards ally agent
        self.waiting = False #True = standing still, waiting for ally agent
        self.avoidwarn = True #True = inform ally of position to avoid (self.avoid)

    def chooseAction(self, vision, msg):

        #ineffective pathing
        #no add to danger if bodyblocked

        highThreshold = 1500
        lowThreshold = 500

        m = self.nutrients["M"]
        s = self.nutrients["S"]

        if (m<self.lastMvalue and s<self.lastSvalue and not self.waiting):
            self.approach -= 1
        else:
            self.approach = self.maxapproach

        self.lastMvalue = m
        self.lastSvalue = s
        
        theirmsg = self.decode(msg)
        mymsg = self.encode("")

        if self.waiting:
            self.waittime -= 1
        if not self.waiting and not self.trading:
            self.cd -= 1

        if self.waittime<=0:
            self.waittime = self.maxwait
            self.waiting = False

        if theirmsg=="wait":
            self.waiting = True
        elif theirmsg=="done":
            self.waiting = False
        elif "waiting" in theirmsg: #waiting,25,35
            self.evalWaiting(theirmsg)
            self.trading = True
            self.cd = self.maxcd
        elif "avoid" in theirmsg: #avoid,11,23
            self.evalAvoid(theirmsg, True)
        elif theirmsg=="drop": #drop
            self.evalAvoid(theirmsg, False)

        bodies = [v for v in vision.bodies if v not in self.body]

        #print(str(self.name)+" got "+str(theirmsg))
        #print(str(self.name)+" avoids "+str(self.avoid))

        if (len(bodies)>0
            and not self.waiting and not self.trading
            and self.calcDist(self.body[0],bodies[0])<=8): #ally is nearby
            mymsg = self.encode(str(self.name)+"/"+str(m)+"/"+str(s)) #P0/1540/1220

## lazy ###########################################################################

        if (m>=highThreshold and s>=highThreshold): #lazy
            if self.evalTrade("lazy",theirmsg,m,s):
                mymsg = self.encode("wait")
            return self.movement(vision,mymsg,None,self.lazysteps)

## M panic ########################################################################
        
        elif m<=lowThreshold: #M panic
            if self.evalTrade("m",theirmsg,m,s):
                mymsg = self.encode("wait")
            return self.movement(vision,mymsg,"M",self.mpanicsteps)

## S panic ########################################################################
        
        elif s<=lowThreshold: #S panic
            if self.evalTrade("s",theirmsg,m,s):
                mymsg = self.encode("wait")
            return self.movement(vision,mymsg,"S",self.spanicsteps)

## normal #########################################################################
        
        else: #normal
            if self.evalTrade("normal",theirmsg,m,s):
                mymsg = self.encode("wait")
            return self.movement(vision,mymsg,None,self.normalsteps)
        
## msg utils #########################################################################

    def encode(self, string):
        return string.encode('utf-8')

    def decode(self, string):
        return string.decode('utf-8')

    def evalTrade(self, strategy, theirmsg, m, s):
        if "/" not in theirmsg:
            return False #not a trade request
        
        v = theirmsg.split("/")
        
        if self.cd>0 or v[0]==self.name or self.waiting or int(v[1])<=0 or int(v[2])<=0:
            return False

        if strategy=="lazy":
            md = m-int(v[1])
            sd = s-int(v[2])
            if (md>0 and sd>0 and md+sd>=250): #donate
                return True
        elif strategy=="normal":
            md = m-int(v[1])
            sd = s-int(v[2])
            if (md>0 and sd>0 and md+sd>=100): #donate
                return True
        elif strategy=="s":
            md = m-int(v[1])
            sd = s-int(v[2])
            if (sd<0 and md<=-sd*2): #donate
                return True
        else:
            md = m-int(v[1])
            sd = s-int(v[2])
            if (md<0 and sd<=-md*2): #donate
                return True

        return False

    def evalWaiting(self, theirmsg):
        if not self.trading:
            s = theirmsg.split(",")
            self.theirX = int(s[1])
            self.theirY = int(s[2])

    def evalAvoid(self, theirmsg, b):
        if b:
            s = theirmsg.split(",")
            self.avoid = (int(s[1]), int(s[2]))
        else:
            self.avoid = (-1,-1)

## action utils #########################################################################
        
    def getClosestFood(self, food, priority):      
        lowestdist = -1
        closestfood = None

        if priority!=None:
            for x,y in food:
                if food[x,y]==priority and (x,y) not in self.danger:
                    dist = abs(self.body[0][0]-x)+abs(self.body[0][1]-y)
                    if lowestdist==-1:
                        lowestdist = dist
                        closestfood = (x,y)
                    elif dist<lowestdist:
                        lowestdist = dist
                        closestfood = (x,y)

            if closestfood!=None:
                return closestfood
            else:
                priority = None
        
        if priority==None:           
            for x,y in food:
                if (x,y) not in self.danger:
                    dist = abs(self.body[0][0]-x)+abs(self.body[0][1]-y)
                    if lowestdist==-1:
                        lowestdist = dist
                        closestfood = (x,y)
                    elif dist<lowestdist:
                        lowestdist = dist
                        closestfood = (x,y)

        return closestfood

    def findPath(self, vision, goal, steps):
        if goal==None:
            return []

        #s = ""
        
        head = self.body[0]
        visited = set()
        
        tiles = [(0, head,[])] #tile = (estimate, point, [actions taken to reach that point])
                               #tiles is a heap
        
        while tiles != [] and steps>0:
            tile = heappop(tiles)
            #s += "picked "+str(tile[0])+" "+str(tile[1])+" at step "+str(steps)+"\n"
            visited.add(tile[1])
            if (tile[1])==goal:
                return tile[2] #finished
            for act in ACTIONS[1:]:
                newpos = self.world.translate(tile[1], act)
                if newpos not in self.world.walls and newpos not in vision.bodies and newpos not in visited:
                    heappush(tiles, (self.getHeuristic(newpos, tile[2]+[act])))
            steps-=1

        #print(s)
        #print("from "+str(head)+" to "+str(goal))
        return []

    def getEstimate(self,point,path): #cost+estimated cost until goal
        if point==None or path==None or self.goal==None:
            return (50, (-1,-1), []) #discard
        x = abs(self.goal[0]-point[0])
        if x>self.world.size[0]/2:
            x = self.world.size[0] - x
        y = abs(self.goal[1]-point[1])
        if y>self.world.size[1]/2:
            y = self.world.size[1] - y
        return (len(path)+x+y, point, path) #returns a tile => (estimate, point, [actions taken to reach that point])

    def getHeuristic(self,point,path): #estimated cost until goal
        if point==None or path==None or self.goal==None:
            return (50, (-1,-1), []) #discard
        x = abs(self.goal[0]-point[0])
        if x>self.world.size[0]/2:
            x = self.world.size[0] - x
        y = abs(self.goal[1]-point[1])
        if y>self.world.size[1]/2:
            y = self.world.size[1] - y
        return (x+y, point, path) #returns a tile => (estimate, point, [actions taken to reach that point])

    def calcDist(self,tile1,tile2): #these tiles are only (x,y) tuples
        x = abs(tile1[0]-tile2[0])
        if x>self.world.size[0]/2:
            x = self.world.size[0] - x
        y = abs(tile1[1]-tile2[1])
        if y>self.world.size[1]/2:
            y = self.world.size[1] - y
        return x+y

    def possible(self,pos,vision,head):
        if pos in self.world.walls or pos in vision.bodies or not self.safeAction(pos,vision,head):
            return False
        else:
            return True

    def safeAction(self, p, vision, lastp=None, iteration=1): #check if the next action doesn't lead into a dead end
        if p in self.danger:
            return False
        
        if p==None: #no possible action
            return False

        if iteration>self.futuresteps:
            return True #assume it has checked enough squares to know
        
        escapes = 0
        nextp = None
        for act in ACTIONS[1:]:
            newpos = self.world.translate(p, act)
            bodies = [b for b in vision.bodies if b not in self.body]
            if newpos!=lastp and newpos not in self.world.walls and newpos not in bodies and newpos not in self.avoid:
                escapes += 1
                if (escapes > 1): #guaranteed to be safe
                    return True                 
                nextp = newpos

        if not self.safeAction(nextp,vision,p,iteration+1): #check a step ahead
            if p not in vision.bodies: 
                self.danger.add(p) #add to cache
            return False
        
        return True

    def hallway(self, head, p, vision):
        escapes = 0
        for act in ACTIONS[1:]:
            newpos = self.world.translate(p, act)
            if newpos not in self.world.walls and newpos not in vision.bodies:
                escapes += 1
                if (escapes > 1): #guaranteed to be safe
                    return False
        return True

    def randomMovement(self, head, vision, msg, steps, maxrange):
        newpos=None
        while (newpos==None or not self.safeAction(newpos,vision)):
            possibilities = [-maxrange-1, -maxrange, -maxrange+1, maxrange-1, maxrange, maxrange-1]
            x = head[0]+random.choice(possibilities)
            if x>self.world.size[0]:
                x = x - self.world.size[0]
            elif x<0:
                x = self.world.size[0] + x
            y = head[1]+random.choice(possibilities)
            if y>self.world.size[1]:
                y = y - self.world.size[1]
            elif y<0:
                y = self.world.size[1] + y
            newpos=(x,y)
        self.currentpath = self.findPath(vision, newpos, steps)
        self.bored = self.maxbored

        return self.movement(vision,msg,None,steps)

    def approachMovement(self, head, vision, msg, steps):
        #     |
        #  1  |  2
        #-----------
        #  3  |  4
        #     |

        newpos = None
        xhalf = self.world.size[0]/2
        yhalf = self.world.size[1]/2

        xtd = floor(abs(head[0]-xhalf)/4)
        ytd = floor(abs(head[1]-yhalf)/4)

        xpossibilities = []
        ypossibilities = []
        
        if (head[0]<=xhalf and head[1]<=yhalf):
            #quad = 1
            xpossibilities = [head[0]+xtd-1,head[0]+xtd,head[0]+xtd+1]
            ypossibilities = [head[1]+ytd-1,head[1]+ytd,head[1]+ytd+1]
        elif (head[0]>=xhalf and head[1]<=yhalf):
            #quad = 2
            xpossibilities = [head[0]-xtd-1,head[0]-xtd,head[0]-xtd+1]
            ypossibilities = [head[1]+ytd-1,head[1]+ytd,head[1]+ytd+1]
        elif (head[0]<=xhalf and head[1]>=yhalf):
            #quad = 3
            xpossibilities = [head[0]+xtd-1,head[0]+xtd,head[0]+xtd+1]
            ypossibilities = [head[1]-ytd-1,head[1]-ytd,head[1]-ytd+1]
        else:
            #quad = 4
            xpossibilities = [head[0]-xtd-1,head[0]-xtd,head[0]-xtd+1]
            ypossibilities = [head[1]-ytd-1,head[1]-ytd,head[1]-ytd+1]

        while (newpos==None or not self.safeAction(newpos,vision)):
            newpos = (random.choice(xpossibilities), random.choice(ypossibilities))

        self.currentpath = self.findPath(vision, newpos, steps)
        self.approach = self.maxapproach #reset

        return self.movement(vision,msg,None,steps)

    def lastresort(self, head, vision, msg):
        validact = []
        
        for act in ACTIONS[1:]:
            newpos = self.world.translate(head, act)
            if self.possible(newpos,vision,head):
                validact.append(act)

        if validact == []:
            self.goingforfood = False
            return ACTIONS[:1][0], msg #rip
        
        self.lastact = random.choice(validact)
        self.bored = self.maxbored
        self.goingforfood = False
        hallway = self.hallway(head, self.world.translate(head, self.lastact), vision)
        if hallway and self.avoidwarn:
            msg = self.encode("avoid,"+str(head[0])+","+str(head[1]))
            self.avoidwarn = False
        if not hallway and not self.avoidwarn:
            msg = self.encode("drop")
            self.avoidwarn = True
        return self.lastact, msg
            
    def movement(self, vision, msg, priority, steps, iteration=1):
        head = self.body[0]
        self.pastturns -= 1

        if iteration>10:
            return self.lastresort(head, vision, msg)

        if self.waiting:
            return ACTIONS[:1][0], self.encode("waiting,"+str(head[0])+","+str(head[1]))

        if self.trading:
            np = None
            xp = [(self.theirX+1,self.theirY),(self.theirX,self.theirY+1),
                  (self.theirX-1,self.theirY),(self.theirX,self.theirY-1)]
            for i in range(0, len(xp)):
                if self.safeAction(xp[i],vision):
                    self.currentpath = self.findPath(vision, xp[i], steps)
                    if self.currentpath == []:
                        self.trading = False
                        return ACTIONS[:1][0], self.encode("done") #give up on trading
                    break            

        if self.currentpath == [] and len(vision.food)>0:
            self.goal = self.getClosestFood(vision.food, priority)
            self.currentpath = self.findPath(vision, self.goal, steps)
            if self.currentpath != []:
                self.goingforfood = True

        #action queue not empty
        if self.currentpath != []:
            action = self.currentpath[0]
            nextPos = self.world.translate(head, action)
            if ((self.possible(nextPos,vision,head) and self.goingforfood) or
            (self.possible(nextPos,vision,head) and not self.goingforfood and len(vision.food)==0)):
                hallway = self.hallway(head, nextPos, vision)
                if hallway and self.avoidwarn:
                    msg = self.encode("avoid,"+str(head[0])+","+str(head[1]))
                    self.avoidwarn = False
                if not hallway and not self.avoidwarn:
                    msg = self.encode("drop")
                    self.avoidwarn = True
                self.currentpath[0:1] = []
                return action, msg
            elif (not self.goingforfood and len(vision.food)>0): #found food but not chasing it, adapt
                self.goal = self.getClosestFood(vision.food, priority)
                self.currentpath = self.findPath(vision, self.goal, steps)
                if self.currentpath != []:
                    self.goingforfood = True
                return self.movement(vision,msg,priority,steps,iteration+1)
            else:
                self.currentpath = []
                
        #escape
        if (self.pastturns<0):
            self.pastturns = self.maxpastturns
            dist = self.calcDist(head,self.past)
            self.past = head
            if (dist<self.mindist):
                self.goingforfood = False
                return self.randomMovement(head,vision,msg,self.escapesteps,self.escaperange)

        if (self.bored==0 and self.lastact!=None): #bored
            self.goingforfood = False
            return self.randomMovement(head,vision,msg,self.escapesteps,self.boredrange)

        if self.approach==0: 
            self.goingforfood = False
            return self.approachMovement(head,vision,msg,self.approachsteps)

        if self.lastact != None:
            nextPos = self.world.translate(head, self.lastact)
            if self.possible(nextPos,vision,head):
                self.bored -= 1
                self.goingforfood = False
                hallway = self.hallway(head, nextPos, vision)
                if hallway and self.avoidwarn:
                    msg = self.encode("avoid,"+str(head[0])+","+str(head[1]))
                    self.avoidwarn = False
                if not hallway and not self.avoidwarn:
                    msg = self.encode("drop")
                    self.avoidwarn = True
                return self.lastact, msg

        return self.lastresort(head, vision, msg)
