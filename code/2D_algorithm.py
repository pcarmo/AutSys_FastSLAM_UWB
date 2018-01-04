'''
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        IMPORT LIBRARIES
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
'''

''' Libraries for Python '''
import numpy as np
import math as mt
import scipy.stats as sps
import copy as cp

''' Libraries for ROS '''
import rospy
import tf
# import msg types
from monarch_uwb.msg import uwb_anchor_array
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray, Pose, Quaternion, Point
from visualization_msgs.msg import MarkerArray, Marker

'''
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    CLASSES DEFINITION
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
'''

class MyPosition(object): # for 2D orientation
    def __init__(self, position_x=0.0, position_y=0.0):
        super(MyPosition, self).__init__()
        self.x = position_x
        self.y = position_y

    ''' Arithmetic Operations '''
    def __add__(self, other): # define function to add positions
        x = self.x + other.x
        y = self.y + other.y
        return MyPosition(x,y)
    def __iadd__(self, other):
        return MyPosition( self.x + other.x, self.y + other.y )
    def __sub__(self, other):
        return MyPosition( self.x - other.x, self.y - other.y )

    '''Class functions'''
    def dist(self, other):
        return mt.sqrt( (self.x-other.x)**2 + (self.y-other.y)**2 )
    def angle2D(self, other):
        return np.arctan2( other.y-self.y, other.x-self.x )
    def matrix(self): # convert to an matrix 2x1
        return np.matrix([self.x, self.y]).T
    def point_ROS(self):
        pointSTD = Point()
        pointSTD.x = self.x
        pointSTD.y = self.y
        return pointSTD

    def __repr__(self):
        return '(x={:+0.2f},y={:+0.2f})'.format(self.x,self.y)

class MyLandmark(object): # 2D space
    def __init__(self, idd=0, mean=MyPosition(), var=1.0): # important to tune this variance
        super(MyLandmark, self).__init__()
        self.id = idd # integer to identify landamark
        self.mean = mean
        self.var = var*np.identity(2)
        #self.marker = Marker()

    def marker_ROS(self,lidd):
        markerSTD = Marker()
        markerSTD.header.stamp = rospy.Time.now()
        markerSTD.header.frame_id = "odom"
        markerSTD.type = Marker.SPHERE
        markerSTD.id = lidd
        markerSTD.pose.position = self.mean.point_ROS()
        markerSTD.pose.orientation.w = 1
        markerSTD.scale.x = self.var.item(0,0) # not to scale
        markerSTD.scale.y = self.var.item(1,1) # not to scale
        #markerSTD.scale.z = self.var.item(2,2) # not to scale
        markerSTD.color.a = 1.0 # set as visible
        markerSTD.color.r = float(self.id%3==0)
        markerSTD.color.g = float(self.id%3==1)
        markerSTD.color.b = float(self.id%3==2)
        return markerSTD

    def __repr__(self):
        return 'Land({}):{}({:0.4f}|{:0.4f})'.format(self.id, self.mean,
                self.var.item((0,0)), self.var.item((1,1)))

class MyPose(object): # REVIEWW ORIENTATION ANGULAR LIMITS
    def __init__(self, position=MyPosition(), orientation=0.0):
        super(MyPose, self).__init__()
        self.pos = position
        self.tt  = orientation

    ''' Arithmetic Operations '''
    def __add__(self, other): # define function to add positions
        pp = self.pos + other.pos
        tt = self.tt + other.tt
        return MyPose(pp, tt)
    def __iadd__(self, other):
        return MyPose( self.pos + other.pos, self.tt + other.tt)
    def __sub__(self, other):
        pp = self.pos - other.pos
        tt = self.tt  - other.tt
        return MyPose(pp, tt)

    '''Class Functions '''
    def transform(self, d): # Transfor positions in same plane
        xT = self.pos.x + d * np.cos(self.tt)
        yT = self.pos.y + d * np.sin(self.tt)
        return MyPosition(xT, yT)
    def pose_ROS(self): # Transform MyPose to standard pose from ROS
        poseSTD = Pose()
        # copy positions (floats) from MyPose to Pose
        poseSTD.position = self.pos.point_ROS()
        # convert orientation from MyPose to Quaternions
        xx, yy, zz, ww = tf.transformations.quaternion_from_euler(0,0,self.tt)
        poseSTD.orientation = Quaternion(xx, yy, zz, ww)
        return poseSTD

    def __repr__(self):
        return '({}|{:+0.2f}deg)'.format(self.pos, self.tt)

class MyParticle(object):
    def __init__(self, pose=MyPose(), mapp=np.array([],object)):
        super(MyParticle, self).__init__()
        self.pose = pose
        self.mapp = mapp # list of landamrks
        self.hypo = [[],[]] # [dist_list per landamark, point_list]

    def dist(self):
        return [MyPosition().dist(l.mean) for l in self.mapp]

    def __repr__(self):
        d = self.dist()
        return 'Part:{}\n({})[{}]\n\tDist:{}'.format(self.pose, self.mapp, len(self.hypo[0]), d)

class MyTag(object):
    """MyTag is a class to interact with an UWB topic"""
    def __init__(self, idd='None', dA=0, dB=0, dC=0):
        super(MyTag, self).__init__()
        self.id = idd
        self.dA = dA
        self.dB = dB
        self.dC = dC

        '''ROS FUNCTIONS'''
    # callback function for subscribing
    def callback(self, ros_data):
        for anchor in ros_data.anchors:
            if anchor.anchor_id == 'A':
                self.dA = anchor.radius
            elif anchor.anchor_id == 'B':
                self.dB = anchor.radius
            else:
                self.dC = anchor.radius
    # define topic to be subscribed
    def subscribe(self, path):
        rospy.Subscriber(path, uwb_anchor_array, self.callback)

    def __repr__(self):
        return 'Tag({}):(dA={:0.4f},dB={:0.4f},dC={:0.4f})'.format(self.id,
                self.dA, self.dB, self.dC)

class MyOdom(object):
    """MyOdom is a class to interact with an odometry topic"""
    def __init__(self, pose=MyPose(), flag=0):
        super(MyOdom, self).__init__()
        self.pose = pose
        self.new = flag
        self.quater = Quaternion()

    def odom_to_tag(self, idd = 'fr'):
        if idd == 'fr':
            d = 0.45
        elif idd == 're':
            d = -0.45
        xx = d*np.cos(self.tt)
        yy = d*np.sin(self.tt)
        x_T = self.pos.x + xx
        y_T = self.pos.y + yy
        return MyPosition(x_T, y_T)

    '''ROS FUNCTIONS'''
    # callback function for subscribing
    def callback(self, ros_data):
        self.new = self.new + 1 # mark that new information was received
        self.pose.pos.x = ros_data.pose.pose.position.x
        self.pose.pos.y = ros_data.pose.pose.position.y

        self.quater = ros_data.pose.pose.orientation
        roll,pitch,yaw = tf.transformations.euler_from_quaternion([self.quater.x,
                                self.quater.y, self.quater.z, self.quater.w])
        self.pose.tt = yaw   # orientation around z
    # define topic to be subscribed
    def subscribe(self, path):
        rospy.Subscriber(path, Odometry, self.callback)

    def __repr__(self):
        return 'Odom({}):{}'.format(self.new, self.pose)


'''
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    FUNCTION DEFINITION
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
'''

def non_colinear(posi_list, posi):
    ''' Receives list of positions and one position
        Returns True if position is non-colinear with all others '''
    threshold_dist = 0.2
    threshold_ang  = 30 * np.pi/180

    if len(posi_list)==0:
        Truth = True
    elif len(posi_list)==1:
        Truth = posi_list[0].dist(posi) > threshold_dist
    elif len(posi_list)==2:
        angle0 = posi_list[0].angle2D(posi_list[1])
        if posi_list[1].dist(posi) > threshold_dist and posi_list[0].dist(posi) > threshold_dist:
            angle1 = posi_list[1].angle2D(posi)
            Truth = abs(angle0-angle1) > threshold_ang
        else:
            Truth = False
    else:
        Truth = False

    return Truth

def distancia(A1,A2,B1,B2):
    return mt.sqrt( (A1-B1)**2 + (A2-B2)**2 )

def frange(start, stop, size):
    i = start
    k = 0
    step = (stop-start)/(size-1)
    vect = np.empty([1,size])
    for k in range(0,size):
        vect[0,k] = start + step * k
        k = k + 1
    return vect

def trilat_3d(dist_vect, pose_mat):

    mesh_size = 15
    num_it = 4
    next = 2.5 #numero de cubos para a malha seguinte (1 = 1 cubo para cada lado do melhor valor )
    N = len(dist_vect)


    matriz = np.empty([N, 2])
    min_coord = np.empty([2])
    max_coord = np.empty([2])
    X = np.empty([2,mesh_size+1])
    beacon = np.empty([2,1])
    step = np.empty([2,1])

    a = 0.0

    k=0
    for pose in pose_mat:
        matriz[k][0] = pose.x
        matriz[k][1] = pose.y
        k=k+1


    max_vect = matriz + dist_vect * np.array([1, 1])
    min_vect = matriz - dist_vect * np.array([1, 1])

    for i in range(0, 3):
        min_coord[i] = min(min_vect[:,i])
        max_coord[i] = max(max_vect[:,i])

    min_coord[3] = max(np.array([0, min_coord[0]]))

    #iterations

    error = float(10**10)
    for i in range(0, num_it):
        step = (max_coord-min_coord)/mesh_size
        for j in range(0,3):
            #print(min_coord, max_coord)
            X[j]=frange(min_coord[j] , max_coord[j] , mesh_size+1)


        for x in X[0]:
            for y in X[1]:
                for z in X[2]:
                    new_error = 0;
                    for k in range(0,N):
                        point_d = distancia(x,y,z,matriz[k,0],matriz[k,1],matriz[k,2])
                        new_error = float(new_error + (dist_vect[k] - point_d)**2)
                    if new_error<error:
                        beacon = np.array([x, y, z])
                        #print('error:', error,'beacon:',beacon)
                        error = new_error


        min_coord = beacon - next * step
        max_coord = beacon + next * step
        min_coord[2] = max(np.array([0, min_coord[0]]))

    return MyPosition(beacon[0], beacon[1], beacon[2])


def EKF_land(mean01, var01, z, pos01, pos):

    ''' EKF Prediction '''
    mean_pred = A*mean01.matrix() # matrix 3x1
    var_pred = A*var01*A.T + R # matrix 3x3

    ''' EKF Update '''
    aa = np.arctan2(mean_pred.item(1) - pos.y,mean_pred.item(0) - pos.x)

    z_est = pos.matrix() + np.matrix([z*np.cos(aa), z*np.sin(aa)]).T
    vr = 5
    rot_z = np.matrix( [ [np.cos(aa), np.sin(-aa)] , [np.sin(aa), np.cos(aa)] ] )
    C = rot_z*np.diag([vr, 20*vr])*rot_z.T


    K = C * (C+var_pred)**(-1)
    mean = z_est + K * (mean_pred - z_est) # matrix 2x1
    var = C - K*C # matrix 2x2

    mean = MyPosition(mean.item(0), mean.item(1)) #convert to myposition

    return mean, var

def fastSLAM(part_set, odom, z_list):

    new_part_set = np.array( [], object) # empty list to be returned
    w_set = np.array( [] ) # particles weight (internal)

    ''' For each particle '''

    for part in part_set:
        new_part = MyParticle() # new particle to be added to set

        ''' Add noise to odometry measure '''
        u_0 = np.sqrt(odom.pos.x**2 + odom.pos.y**2)
        u_x = u_0*np.cos(part.pose.tt) + np.random.normal(0,sigma_odom)
        u_y = u_0*np.sin(part.pose.tt) + np.random.normal(0,sigma_odom)
        u_tt = odom.tt   + np.random.normal(0,sigma_odomtt)
        u = MyPose( MyPosition(u_x, u_y), u_tt)
        #print('Odometry + Noise:',u)


        ''' STEP 1 - RETRIEVAL (get old pose) '''
        pose01 = part.pose              # particle old pose
        posF01 = pose01.transform(dF)   # front tag old position

        ''' STEP 2 - PREDICTION (estimate new pose) '''
        pose = pose01 + u           # particle new pose
        posF = pose.transform(dF)   # front tag new position
        new_part.pose = pose

        ''' STEP 3 - UPDATEE (updatee landamark position) '''
        new_map = np.array([],object)

        for land, z in zip( part.mapp, z_list[0]):

            ''' Call EKF '''
            mean01, var01 = land.mean, land.var
            #print('EKF input:',mean01, var01)
            mean, var = EKF_land(mean01, var01, z, posF01, posF)
            #print('EKF output:',mean, var)
            new_land = MyLandmark(land.id, mean, var)
            #print('New Landmark:',new_land)

            ''' Save changes to map '''
            new_map = np.append(new_map, new_land)

        new_part.mapp = new_map
        #print('New Map:',new_part.mapp)

        ''' STEP 4 - WEIGHT (assign new importance weight) '''
        aux_w = np.array([])

        for land, zF, zR in zip( new_part.mapp, z_list[0], z_list[1]):

            miu = land.mean.dist(posF)
            sigma = sigma_UWB
            probF = sps.norm(miu, sigma).pdf(zF)

            posR = pose.transform(dR)   # rear tag new position
            miu = land.mean.dist(posR)
            sigma = sigma_UWB
            probR = sps.norm(miu, sigma).pdf(zR)

            prob = probF * probR

            aux_w = np.append(aux_w, prob)

        w = np.prod(aux_w) # independent observations


        ''' Save changes to particle and weight sets '''
        new_part_set = np.append( new_part_set, cp.deepcopy(new_part))
        w_set = np.append( w_set, w )


    ''' Selective resampling '''

    w_setN = w_set / np.sum(w_set) # normalize weights
    index = np.argmax(w_setN) #
    best_part = cp.deepcopy(new_part_set[index])

    N_eff = 1/np.sum(w_setN**2) # number of efective particles

    # Decide if resampling should be done
    if N_eff < N/2:
        #print('Resampling occured because N_eff=',N_eff)
        new_part_set = np.random.choice(new_part_set, N, p=w_setN)
    else:
        #print('No resampling needed N_eff=',N_eff)
        pass


    return new_part_set, best_part

def publish_results(part_set, pub_part, pub_land):
    poseArray = PoseArray()
    landArray = MarkerArray()

    poseArray.header.stamp=rospy.Time.now()
    poseArray.header.frame_id="odom"
    idd = 0
    for part in part_set:
        poseSTD = part.pose.pose_ROS() # convert MyPose to Pose (standard from ROS)
        poseArray.poses.append(poseSTD) # append standard Pose to PoseArray
        for land in part.mapp:
            markerSTD = land.marker_ROS(idd)
            landArray.markers.append(markerSTD)
            idd += 1

    pub_part.publish(poseArray)
    pub_land.publish(landArray)


'''
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        MAIN FUNCTION
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
'''

if __name__ == '__main__':
    print("New Algorithm 2D is running...")

    # ============   CONSTANT DEFINITION STARTS HERE   ============
    ''' Code constants'''
    N = 20 # number of particles
    # Kalman matrixes
    A = np.identity(2)
    R = 0.008*np.identity(2) # Process covariance noise (tunable)

    ''' Real World Constants '''
    z_T =  0.237 # z coordenate of tags
    dF  =  0.450 # distance to front tag (only in x)
    dR  = -0.450 # distance to rear tag  (only in x)
    # Sensors errors
    sigma_odom   = 0.05
    sigma_odomtt = 3 * np.pi/180
    sigma_UWB    = 0.15
    # ============   CONSTANT DEFINITION ENDS HERE   ============


    # ============   INIITIALIZATION STARTS HERE   ============
    ''' ROS initialization '''
    rospy.init_node('PythonScript')
    # topics published from this node (topic_name, topic_type, queue_size)
    pub_particles = rospy.Publisher('/particles', PoseArray, queue_size=100)
    pub_landmarks = rospy.Publisher('/landmarks', MarkerArray, queue_size=100)
    # topics subscribed from this node (topic_name)
    sub_odom = MyOdom()                         # odometry subscriber
    sub_odom.subscribe('/RosAria/pose')
    sub_tagF = MyTag('front')                   # front tag subscriber
    tagF_topic = '/front_uwb_driver_node/front_tag_readings'
    sub_tagF.subscribe(tagF_topic)
    sub_tagR = MyTag('rear')                    # rear tag subscriber
    tagR_topic = '/rear_uwb_driver_node/rear_tag_readings'
    sub_tagR.subscribe(tagR_topic)

    ''' Particle initialization '''
    pose0 = MyPose()
    r0 = MyPosition(1.5, 0.6)
    lB = MyPosition(0, 0) #bag04
    lC = MyPosition(0, 0) #bag04
    lA = MyPosition(0, 0) #bag04
    mapp0 = np.array([MyLandmark(0,lA,1000), MyLandmark(1,lB,1000), MyLandmark(2,lC,1000)],object)
    part0 = MyParticle(pose0, mapp0)
    p_set = np.full(N, part0, object)
    new_l = [[],[[],[],[]]]

    flag,num = 1,0
    time_list=[1513268172, 1513268209, 1513268263, 1513268307, 1513268330, 1513268343]
    # ============   INIITIALIZATION ENDS HERE   ============


    # Wait for messages from publisher topics
    print("Waiting for publishers...")
    rospy.wait_for_message(tagF_topic, uwb_anchor_array)
    print("Messages received! Working on it...")

    while True:

        # ============   START WAITING FOR TOPICS  ============
        try:
            rospy.wait_for_message(tagF_topic, uwb_anchor_array, 30)
        except rospy.exceptions.ROSException:
            print("Too much time passed without messages")
            break
        # ============   STOP WAITING FOR TOPICS  ============

        # ============   READINGS STARTS HERE  ============
        odom = cp.deepcopy(sub_odom) # deepcopy because a class (MyOdom) is mutuable
        sub_odom.new = 0 # set flag back to zero
        z_front = [sub_tagF.dA, sub_tagF.dB, sub_tagF.dC] # no deepcopy because each dX is imutable
        z_rear  = [sub_tagR.dB, sub_tagR.dC, sub_tagR.dA]
        z = [z_front, z_rear]

        move = odom.pose - pose0
        pose0 = cp.deepcopy(odom.pose) # deepcopy because a class (MyPose) is mutuable
        # ============   READINGS ENDS HERE  ============

        ''' Call fastSLAM Algorithm '''
        # ============   LANDAMRK INIT STARTS HERE  ============
        if False:#p_set[0].mapp.size != len(z_front): #new_landamarks
            posiF = pose0.transform(dF)
            if non_colinear(new_l[0], posiF): # only save non-colinear
                new_l[0].append(posiF)
                new_l[1][0].append(z_front[0])
                new_l[1][1].append(z_front[1])
                new_l[1][2].append(z_front[2])
                print "I received one point. I have now:", len(new_l[0])
                if len(new_l[0])>=3: #perform trilateration with three or more points
                    print "I am trying trilateration with 3 points, wish me luck..."
                    land0 = trilat_3d(np.matrix(new_l[1][0]).T, new_l[0])
                    land1 = trilat_3d(np.matrix(new_l[1][1]).T, new_l[0])
                    land2 = trilat_3d(np.matrix(new_l[1][2]).T, new_l[0])
                    mapp = np.array([MyLandmark(0,land0), MyLandmark(1,land1), MyLandmark(2,land2)],object)
                    new_l = [[],[[],[],[]]] #reset
                    for par in p_set:
                        par.mapp = cp.deepcopy(mapp)
            else:
                pass
        # ============   LANDAMRK INIT ENDS HERE  ============
        p_set, p_best = fastSLAM(p_set, move, z)

        # ============   Print given points START HERE  ============
        now = rospy.get_rostime()
        if (now.secs in time_list) and flag:
            num+=1
            print(num)
            print(p_best)
            flag = 0
        elif now.secs not in time_list:
            flag = 1
        publish_results(p_set, pub_particles, pub_landmarks)
        # ============   Print given points END HERE  ============

    print(p_set)
    print("Best Particle:", p_best)
    print("Front Tag:", z_front)
