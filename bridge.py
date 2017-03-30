"""
"Method of Joints" 2D Truss Solver and Visualiser
Billy Woods 30/3/2017

Dependencies:
*numpy
*pygame
*python2.x (although should work in python3 also, but un-tested)

Developed to aid a first-year engineering spaghetti bridge project. Make of
this code and the included sample bridge as you wish, but please share derivatives.

Bridges are inputted as a list of (x,y) points, which represent the position
of each joint in the truss; and a list of members, represented as a 
start point and endpoint, referencing indices in the aforementioned list 
of joints.

Loads, and the points at which supports (two), are positioned also have to be
specified. Refer to the example bridge, b1, to get the gist of how this is done.

"""

import numpy
import math
import pygame

# global vars/defines
SCREEN_SIZE = (1000,400)
FONT_SIZE = 13
BACKGROUND_COLOUR = (255,255,255)


class load(object):
    """
    Point is an index, referring to list of defined points/joints.
    Angle is measured from positive x axis, going anti clockwise.
    Magnitude is positive scalar.
    """
    def __init__(self, point, angle, magnitude):
        self.point = point
        self.angle = angle
        self.magnitude = magnitude

    def get_x_comp(self):
        return math.cos(self.angle) * self.magnitude
    
    def get_y_comp(self):
        return math.sin(self.angle) * self.magnitude

class member(object):
    def __init__(self, start, end, solved = False):
        self.start = start
        self.end = end
        self.force = 0
        self.solved = solved

    def __str__(self):
        compOrTension = ""
        if self.force < 0:
            compOrTension = "compression"
        else:
            compOrTension = "tension"
        return "{}N {} @ member from {} to {}".format(abs(round(self.force,3)),compOrTension, self.start, self.end)

        
class bridge(object):
    """
    _roller and _pin are passed pseudo-"by reference".
    They specify the index of the point at which they are
    located.

    """
    def __init__(self, _points, _members, _loads, _roller, _pin):
        self.points = _points
        self.members = [member(m[0], m[1]) for m in  _members]
        self.loads = _loads
        self.rollerSupp = _roller
        self.pinSupp = _pin
        self.solved = False

    def is_determinate(self):
        # 2*joints = members + the three support reactions
        return (len(b1Points)*2) == (len(b1Members) + 3)

    def calculate_moment(self, refPoint, load):
        momentPoint = self.points[refPoint]
        loadPoint = self.points[load.point]
        distDirect = math.sqrt(pow(momentPoint[0] - loadPoint[0],2) + pow(momentPoint[1] - loadPoint[1],2))
        theta = -( math.atan2(loadPoint[1] - momentPoint[1], loadPoint[0] - momentPoint[0])) + load.angle 
        perpForce = math.sin(theta) * load.magnitude
        """
        print("point: [{}, {}]\n".format(momentPoint[0], momentPoint[1]))
        print("point: [{}, {}]\n".format(loadPoint[0], loadPoint[1]))
        print("distdirect: {}\ntheta: {}\nmoment: {}\n".format(distDirect, theta, perpForce * distDirect))
        """
        return perpForce * distDirect
    
    """
    assumes all supports are placed so that their force vectors are orthogonal
    must be run before joints can be solved
    """
    def solve_support_reactions(self):
        #moment around pin support = 0
        momentsAtPin = sum([self.calculate_moment(self.pinSupp, l) for l in self.loads])
        supportXDist = self.points[self.rollerSupp][0] - self.points[self.pinSupp][0]
        rollerReaction = -momentsAtPin/supportXDist
        #print("roller reaction: {}, {}\n".format(math.asin(rollerReaction/abs(rollerReaction)), abs(rollerReaction)))
        self.loads.append(load(self.rollerSupp, math.asin(rollerReaction/abs(rollerReaction)), abs(rollerReaction)))
        
        #calculate pin support reaction
        # Fy = 0, Fx = 0
        unbalX = 0
        unbalY = 0
        for l in self.loads:
            unbalX += l.get_x_comp()
            unbalY += l.get_y_comp()

        
        print("pin reaction: x:{}kN, y:{}kN\n".format(round(-unbalX,3), round(-unbalY,3)))
        #print(momentsAtPin)
        print("roller reaction: x:0kN, y:{}kN\n".format(rollerReaction))
        
        self.loads.append(load(self.pinSupp, math.atan2(-unbalY, -unbalX), math.sqrt(pow(unbalX,2) + pow(unbalY,2))))
        #with the support reactions added as loads, our bridge is in
        # equilibrium and ready for method of joints

    
    def get_members_from_point(self, point):
        membersByIndex = []
        for i,m in enumerate(self.members):
            if point == m.start or point == m.end:
                membersByIndex.append(i)
        return membersByIndex

    def is_point_solvable(self, point):
        return len([x for x in self.get_members_from_point(point) if self.members[x].solved == False]) <= 2

    def is_point_solved(self, point):
        return len([x for x in self.get_members_from_point(point) if self.members[x].solved == False]) == 0
        
    # assumes point is solvable (only two unkowns)
    def solve_point(self, point):
        
        if not self.is_point_solvable(point):
            return False
            
        # apply any external forces which may be present
        unbalX = 0
        unbalY = 0
        for ld in self.loads:
            if ld.point == point:
                unbalX += ld.get_x_comp()
                unbalY += ld.get_y_comp()

        # get angles of all members connected to this point
        members = self.get_members_from_point(point)
        unsolvedMembers = []
        for mem in members:
            m = self.members[mem]
            #calculate member angle relative to this point
            memberAngle = 0
            if m.start == point:
                memberAngle = (math.atan2(self.points[m.end][1] - self.points[m.start][1], self.points[m.end][0] - self.points[m.start][0]))
            else:
                memberAngle = (math.atan2(self.points[m.start][1] - self.points[m.end][1], self.points[m.start][0] - self.points[m.end][0]))
            
            #apply force if member is solved
            if m.solved:
                unbalX += math.cos(memberAngle) * m.force
                unbalY += math.sin(memberAngle) * m.force
            else:
                # (index in global member array, relative member angle)
                unsolvedMembers.append((mem, memberAngle))
            
        if len(unsolvedMembers) == 2:
            # solve for magnitude of each of the two unknowns
            a = numpy.array([[math.cos(unsolvedMembers[0][1]), math.cos(unsolvedMembers[1][1])],
                             [math.sin(unsolvedMembers[0][1]), math.sin(unsolvedMembers[1][1])]])
            b = numpy.array([[-unbalX], [-unbalY]])
            c = numpy.linalg.solve(a,b)
            self.members[unsolvedMembers[0][0]].force = c[0][0]
            self.members[unsolvedMembers[1][0]].force = c[1][0]
            self.members[unsolvedMembers[0][0]].solved = True
            self.members[unsolvedMembers[1][0]].solved = True
        else:
            # solve for magnitude of single unknown
            self.members[unsolvedMembers[0][0]].force = -unbalX/math.cos(unsolvedMembers[0][1])
            self.members[unsolvedMembers[0][0]].solved = True
            if round(self.members[unsolvedMembers[0][0]].force,4) == round(-unbalY/math.sin(unsolvedMembers[0][1]),4):
                print("eq's match! good...")
            else:
                print("eq's don't match! Error with solving...")
                quit()

        return True

    def solve_all(self):
        if not self.is_determinate():
            print("indeterminate!")
            print("joints: {}\nmembers: {}\nreactions: {}\n".format(len(b1Points), len(b1Members), 3))
            return False

        self.solve_support_reactions()
        unsolvedPoints = True
        while unsolvedPoints:
            for i,p in enumerate(self.points):
                if self.is_point_solvable(i) and not(self.is_point_solved(i)):
                    self.solve_point(i)

            # check if we have solved all points, so we don't loop again
            unsolvedPoints = False
            for i,p in enumerate(self.points):
                if not self.is_point_solved(i):
                    unsolvedPoints = True
        self.solved = True
        return True

    def get_member_endpoint_coords(self, memberIndex, scaleFactor):
        p1 = [self.points[self.members[memberIndex].start][0] * scaleFactor,
              self.points[self.members[memberIndex].start][1] * scaleFactor]
        p2 = [self.points[self.members[memberIndex].end][0] * scaleFactor,
              self.points[self.members[memberIndex].end][1] * scaleFactor]
        return [p1, p2]

    def draw_image_to_screen(self, screen, lineThickness, colourScheme, scale, offset):
        pygame.font.init()
        labelFont = pygame.font.SysFont('Arial Black', FONT_SIZE)
        screen.fill(BACKGROUND_COLOUR)
        # draw members
        for i,m in enumerate(self.members):
            if round(m.force,1) == 0:
                colour = colourScheme[1]
            elif m.force < 0:
                colour = colourScheme[0]
            elif m.force > 0:
                colour = colourScheme[2]

            [startPoint, endPoint] = self.get_member_endpoint_coords(i, scale)
            # convert points to integer, flip y axis as pygame assumes (0,0) at top left
            endPoint = [int(endPoint[0] + offset[0]), int(SCREEN_SIZE[1] - (endPoint[1] + offset[1]))]
            startPoint = [int(startPoint[0] + offset[0]), int(SCREEN_SIZE[1] - (startPoint[1] + offset[1]))]
            #print("sp: ({},{})\nep: ({},{})\n".format(startPoint[0],startPoint[1],
            #        endPoint[0], endPoint[1]))
            pygame.draw.line(screen, colour, startPoint, endPoint, lineThickness)

            labelText = "{}N".format(round(abs(m.force),2))
            labelPos = ((endPoint[0] + startPoint[0])/2 - int(FONT_SIZE * len(labelText)/3),
                       (endPoint[1] + startPoint[1])/2)
            forceLabel = labelFont.render(labelText, 1, (0,127,255))
            screen.blit(forceLabel, labelPos)
    
    def print_diagnostics(self):
        # print force in all members
        for mem in self.members:
            print(mem)

        # find and print max tension and compression
        allForces = [x.force for x in self.members]
        maxTension = max(allForces)
        maxCompression = abs(min(allForces))
        print("\nhighest tension: {}N\nhighest compression: {}N\n".format(maxTension,abs(maxCompression)))
        

if __name__ == "__main__":
    # [x pos, y pos]
    b1Points = [
        [0,0.2],
        [0.1,0.2],
        [0.225,0.2],
        [0.325,0.2],
        [0.425,0.2],
        [0.525,0.2],
        [0.625,0.2],
        [0.725,0.2],
        [0.85, 0.2],
        [0.85,0],

        [0.1, 0.3],
        [0.225,0.3],
        [0.325,0.3],
        [0.425,0.3],
        [0.525,0.3],
        [0.625,0.3],
        [0.725,0.3],
    ]
    # [start point, end point]
    b1Members = [
        [0,1],
        [1,2],
        [2,3],
        [3,4],
        [4,5],
        [5,6],
        [6,7],
        [7,8],
        [7,9],
        [8,9],

        [10,11],
        [11,12],
        [12,13],
        [13,14],
        [14,15],
        [15,16],

        [1,10],
        [2,11],
        [3,12],
        [4,13],
        [5,14],
        [6,15],
        [7,16],

        [0,10],
        [10,2],
        [11,3],
        [12,4],
        [4,14],
        [5,15],
        [6,16],
        [16,8],
    ]

    # used for self-weight calcs
    bridge_weight = 0.1*10 # 100 grams in Newtons

    b1Loading = [
        # (joint num/point, direction, magnitude)

        # self-weight loading (approximate)
        load(10, -math.pi/2, bridge_weight/7),
        load(11, -math.pi/2, bridge_weight/7),
        load(12, -math.pi/2, bridge_weight/7),
        load(13, -math.pi/2, bridge_weight/7),
        load(14, -math.pi/2, bridge_weight/7),
        load(15, -math.pi/2, bridge_weight/7),
        load(16, -math.pi/2, bridge_weight/7),

        # actual load loading
        load(12,-math.pi/2,1.5),
        load(13,-math.pi/2,3),
        load(14,-math.pi/2,1.5),
    ]

    #(points, members, loads (not including support reactions), 
    # index of point/joint which roller support is at, 
    # index of point/joint which pin support is at)
    b1 = bridge(b1Points, b1Members, b1Loading, 0, 9)

    # check for solvability with method of joints, then solve
    if not b1.is_determinate():
        print("indeterminate!")
        print("joints: {}\nmembers: {}\nreactions: {}\n".format(len(b1Points), len(b1Members), 3))
        quit()

    b1.solve_all()

    if b1.solved:
        b1.print_diagnostics()

        # create a visualisation with pygame
        screen = pygame.display.set_mode(SCREEN_SIZE)
        quit = False
        drawn = False
        while not quit:
            event = pygame.event.poll()
            if event.type == pygame.QUIT:
                quit = True

            # static image, so only draw once
            if not drawn:
                drawn = True
                # (screen, line thickness, [compression colour, no force colour, tension colour],
                # scale/size multiplier, [offset x, offset y])
                b1.draw_image_to_screen(screen, 3, [(255,0,0), (0,0,0), (0,255,0)],
                                        1000, [50,50])
                pygame.display.flip()
    else:
        print("Error: bridge could not be solved!\n")
