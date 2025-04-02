#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Path
import tf.transformations
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped


# 用于存储路径的 x, y, yaw 信息
path_data = {
    'x': [],
    'y': [],
    'yaw': []
}
class BS_curve(object):

    def __init__(self,n,p,cp=None,knots=None):
        self.n = n # n+1 control points >>> p0,p1,,,pn
        self.p = p
        if cp:
            self.cp = cp
            self.u = knots
            self.m = knots.shape[0]-1 # m+1 knots >>> u0,u1,,,nm
        else:
            self.cp = None
            self.u = None
            self.m = None

        self.paras = None


    def check(self):
        if self.m == self.n + self.p + 1:
            return 1
        else:
            return 0


    def coeffs(self,uq):
        #计算的是 B 样条基函数
        # n+1 control points >>> p0,p1,,,pn
        # m+1 knots >>> u0,u1,,,nm
        # algorithm is from https://pages.mtu.edu/~shene/COURSES/cs3621/NOTES/spline/B-spline/bspline-curve-coef.html
    
        #N[] holds all intermediate and the final results
        # in fact N is longer than control points,this is just to hold the intermediate value
        # at last, we juest extract a part of N,that is N[0:n+1]
        N = np.zeros(self.m+1,dtype=np.float64) 

        # rule out special cases Important Properties of clamped B-spline curve
        if uq == self.u[0]:
            N[0] = 1.0
            return N[0:self.n+1]
        elif uq == self.u[self.m]:
            N[self.n] = 1.0
            return N[0:self.n+1]

        # now u is between u0 and um
        # first find k uq in span [uk,uk+1)
        check = uq - self.u
        ind = check >=0
        k = np.max(np.nonzero(ind))
        sk = np.sum(self.u==self.u[k])

        N[k] = 1.0 # degree 0
        # degree d goes from 1 to p
        for d in range(1,self.p+1):
            r_max = self.m - d - 1 # the maximum subscript value of N in degree d,the minimum is 0
            if k-d >=0:
                if self.u[k+1]-self.u[k-d+1]:
                    N[k-d] = (self.u[k+1]-uq)/(self.u[k+1]-self.u[k-d+1])*N[k-d+1] #right (south-west corner) term only
                else:
                    N[k-d] = (self.u[k+1]-uq)/1*N[k-d+1] #right (south-west corner) term only

            for i in range(k-d+1,(k-1)+1):
                if i>=0 and i<=r_max:
                    Denominator1 = self.u[i+d]-self.u[i]
                    Denominator2 = self.u[i+d+1]-self.u[i+1]
                    # 0/0=0
                    if Denominator1 == 0:
                        Denominator1 = 1
                    if Denominator2 == 0:
                        Denominator2 = 1

                    N[i] = (uq-self.u[i])/(Denominator1)*N[i]+(self.u[i+d+1]-uq)/(Denominator2)*N[i+1]

            if k <= r_max:
                if self.u[k+d]-self.u[k]:
                    N[k] = (uq-self.u[k])/(self.u[k+d]-self.u[k])*N[k]
                else:
                    N[k] = (uq-self.u[k])/1*N[k]

        return N[0:self.n+1]


    def De_Boor(self,uq):
        # 通过给定的控制点和节点向量，求解 B 样条曲线在某个参数值 u 对应的点
        # Input: a value u
        # Output: the point on the curve, C(u)

        # first find k uq in span [uk,uk+1)
        check = uq - self.u
        ind = check >=0
        k = np.max(np.nonzero(ind))
        
        # inserting uq h times
        if uq in self.u:
            # sk >>> multiplicity of u[k]
            sk = np.sum(self.u==self.u[k])
            h = self.p - sk
        else:
            sk = 0
            h = self.p

        # rule out special cases
        if h == -1:
            if k == self.p:
                return np.array(self.cp[0])
            elif k == self.m:
                return np.array(self.cp[-1])


        # initial values of P(affected control points) >>> Pk-s,0 Pk-s-1,0 ... Pk-p+1,0
        P = self.cp[k-self.p:k-sk+1]
        P = P.copy()
        dis = k-self.p # the index distance between storage loaction and varibale i
        # 1-h
        
        for r in range(1,h+1):
            # k-p >> k-sk
            temp = [] # uesd for Storing variables of the current stage
            for i in range(k-self.p+r,k-sk+1):
                a_ir = (uq-self.u[i])/(self.u[i+self.p-r+1]-self.u[i])
                temp.append((1-a_ir)*P[i-dis-1]+a_ir*P[i-dis])
            P[k-self.p+r-dis:k-sk+1-dis] = np.array(temp)
        # the last value is what we want
        return P[-1]


    def bs(self,us):
        y = []
        for x in us:
            y.append(self.De_Boor(x))
        y = np.array(y)
        return y


    def estimate_parameters(self,data_points,method="centripetal"): 
        #//根据路径点之间的距离来计算出每个点对应的参数值。这个方法通过计算各点之间的累积距离，标准化为参数区间 [0,1]
        #在曲线的定义域 [0, 1] 上均匀分布或按比例分布每个路径点。这些参数值将路径点映射到曲线的参数区间，从而能够描述路径点相对于曲线的位置。
        pts = data_points.copy()
        N = pts.shape[0]
        w = pts.shape[1]
        Li = []
        for i in range(1,N):
            Li.append(np.sum([pts[i,j]**2 for j in range(w)])**0.5)
        L = np.sum(Li)

        t= [0]
        for i in range(len(Li)):
            Lki = 0
            for j in range(i+1):
                Lki += Li[j]
            t.append(Lki/L)
        t = np.array(t)
        self.paras = t
        ind = t>1.0
        t[ind] = 1.0
        return t


    def get_knots(self,method="average"):

        knots = np.zeros(self.p+1).tolist()

        paras_temp = self.paras.copy()
        # m = n+p+1
        self.m = self.n + self.p + 1
        # we only need m+1 knots
        # so we just select m+1-(p+1)-(p+1)+(p-1)+1+1  paras to average
        num = self.m - self.p  # select n+1 paras

        ind = np.linspace(0,paras_temp.shape[0]-1,num)
        ind = ind.astype(int)
        paras_knots = paras_temp[ind]

        for j in range(1,self.n-self.p+1):
            k_temp = 0
            # the maximun of variable i is n-1
            for i in range(j,j+self.p-1+1):
                k_temp += paras_knots[i]
            k_temp /= self.p
            knots.append(k_temp)  #实际knots size=m ,j从1开始， knots[0]=0


        add = np.ones(self.p+1).tolist() 
        knots = knots + add  #最后添加p+1个1    
        knots = np.array(knots)
        self.u = knots
        self.m = knots.shape[0]-1
        return knots


    def approximation(self,pts):

        #根据路径点来计算控制点
        #1：首先使用 paras 计算出的参数值，结合 coeffs 函数计算每个路径点的 B 样条基函数值。
        #2  然后，最小二乘法通过这些基函数和路径点，求解出控制点的位置。控制点数 n+1 被指定为 100（即 n=100），因此你最终得到 100 个控制点。
        ## Obtain a set of parameters t0, ..., tn
        #pts_paras = self.estimate_parameters(pts)
        ## knot vector U;
        #knots = self.get_knots()
        num = pts.shape[0]-1 # (num+1) is the number of data points

        P = np.zeros((self.n+1,pts.shape[1]),dtype=np.float64) # n+1 control points
        P[0] = pts[0]
        P[-1] = pts[-1]

        # compute N
        N = []
        for uq in self.paras:
            N_temp = self.coeffs(uq)  #计算每个路径点的 B 样条基函数值
            N.append(N_temp)
        N = np.array(N)   #N 是一个维度为 (num_points, n+1) 的矩阵，其中 num_points 是路径点的数量，n+1 是控制点的数量

        Q = [0] # hold the location
        for k in range(1,num-1+1):  #1-num-1 
            Q_temp = pts[k] - N[k,0]*pts[0] - N[k,self.n]*pts[-1] 
             # 将第 k 个路径点 pts[k] 从 B 样条的影响中分离出首尾控制点对其的影响，从而能够对中间控制点的位置进行求解 
             #直接 减去首尾点乘对应基函数，因为B样条曲线每个点都是累加的，所以直接减去首尾点乘对应基函数，得到中间控制点的位置
            Q.append(Q_temp)

        b = [0]
        for i in range(1,self.n-1+1):  #n-1 个中间控制点（即 n+1 总控制点减去第一个和最后一个固定控制点）
            b_temp = 0
            for k in range(1,num-1+1):
                b_temp += N[k,i]*Q[k]
            b.append(b_temp)

        b = b[1::]
        b = np.array(b)

        N = N[:,1:(self.n-1)+1]  #提取 N 矩阵中的中间控制点部分
        A = np.dot(N.T,N)
        cpm = np.linalg.solve(A,b)  #通过最小二乘法，我们可以求解控制点 P_0, P_1, ..., P_n，这些控制点使得B样条曲线尽可能逼近路径点。
        P[1:self.n] = cpm  #得到中间的控制点 cpm（即控制点 P[1:n] 的位置）
        self.cp = P
        return P
    
    def set_n(self,n):
        self.n = n


def path_callback(msg):
    global path_data
    # 清空之前的数据
    path_data['x'].clear()
    path_data['y'].clear()
    path_data['yaw'].clear()

    for pose_stamped in msg.poses:
        # 提取 x 和 y
        x = pose_stamped.pose.position.x
        y = pose_stamped.pose.position.y
        
        # 提取四元数并计算 yaw 角
        orientation_q = pose_stamped.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(orientation_list)

        # 将数据保存到列表中
        path_data['x'].append(x)
        path_data['y'].append(y)
        path_data['yaw'].append(yaw)

    rospy.loginfo("Received path with {} points.".format(len(msg.poses)))

def Bs(bs,flag):
    num=len(path_data['x'])
    if flag:
        bs.set_n(num)
    else:
        if num<100:
            bs.set_n(num)
        else:
            bs.set_n(int(num*1/2))
    xx=np.array( path_data['x'])
    yy=np.array( path_data['y'])
    # fig = plt.figure(figsize=(10,5))
    # ax = fig.add_subplot(111)
    # ax.plot(xx,yy)

    
    data = np.array([xx,yy]).T
    paras = bs.estimate_parameters(data)
    knots = bs.get_knots()
    if bs.check():
        cp = bs.approximation(data)

    uq = np.linspace(0,1,num)
    y = bs.bs(uq)
    path=Path()
    path.header = Header()
    path.header.stamp = rospy.Time.now()  # 当前时间
    path.header.frame_id = 'map'  # 假设路径相对于map坐标系
    for i in range(len(y[:,0])):
        # 创建一些PoseStamped对象来表示路径上的点
        pose1 = PoseStamped()
        pose1.pose.position.x = y[i,0]
        pose1.pose.position.y = y[i,1]
        pose1.pose.position.z = 0.0
        pose1.pose.orientation.w = 1.0  # 默认朝向

        # 将PoseStamped对象添加到Path对象中
        path.poses.append(pose1)
    return path
    # ax.plot(y[:,0],y[:,1],'-r')
    # ax.plot(cp[:,0],cp[:,1],'*')
    # # ax.scatter(cp[:,0],cp[:,1])
    # plt.show()


def main():
    # 初始化节点
    
    rospy.init_node('bs_curve')
    pub100 = rospy.Publisher('Bs_k_path', Path, queue_size=10)
    puball = rospy.Publisher('Bs_allpoints_path', Path, queue_size=10)
    # 订阅 path 话题
    rospy.Subscriber("/run_hybrid_astar/searched_path", Path, path_callback)

    bs = BS_curve(0,3)  #怎么能从所有路径点中选出100个控制点并且反解出呢？  求解线性方程反解时，N是 num_points行, n+1列的矩阵用于最小二乘求解 n+1个系数 即 控制点
                        #数学推导最小二乘就理解了，列数决定的。
    bsall=BS_curve(0,3)
    rate = rospy.Rate(1000)  # 10 Hz
    while not rospy.is_shutdown():
        if len(path_data['x'])==0:
            continue

        msg100 = Bs(bs,0)
        msgall = Bs(bsall,1)
        pub100.publish(msg100)
        puball.publish(msgall)
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    
