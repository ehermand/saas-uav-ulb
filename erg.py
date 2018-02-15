"""
Explicit Reference Governor Class

@author: elie
"""

from collections import namedtuple

from math import sqrt
import numpy as np

StateSpace = namedtuple("StateSpace", "A B C D")

class LinearConstraint():
    """
    Linear constraint of the form: a.T*x + b >= 0
    """
    def __init__(self,a,b):
        self.a = a
        self.b = b
        
    def Lyapunov_threshold(self,P,v):
        xv = np.concatenate((v,[[0],[0],[0]]),axis=0)
        return ((self.a.T@xv + self.b)**2)/(self.a.T@P.I@self.a)
        
class WallConstraint(LinearConstraint):
    
    def __init__(self,cW,dW,delta,zeta):
        """
        parameters:
            cW, dW - wall constraint parameters (c vector, d scalar)
            delta - static safety margin (>0)
            zeta - influence region of the wall constraint (>delta)
        """
        norm = np.linalg.norm(cW)
        
        self.cW = cW/norm
        self.dW = dW/norm
        self.delta = delta
        self.zeta = zeta
        
        a = np.concatenate((self.cW,[[0],[0],[0]]),axis=0)
        b = self.dW
        
        super().__init__(a,b)
        
    def rho0(self,v):
        return self.cW*max((self.zeta-(self.cW.T@v + self.dW)) / (self.zeta-self.delta),0)


class SphereConstraint(WallConstraint):
    
    def __init__(self,p0,R,delta,zeta):
        super().__init__(np.array([[1],[0],[0]]),0,delta,zeta)
        
        self.p0 = p0
        self.R = R
        self.dist = 0
        
    def update(self,v):
        c = (self.p0 - v)
        norm = np.linalg.norm(c)
        self.cW = c/norm
        self.dW = norm - self.cW.T@v - self.R
        self.a = np.concatenate((self.cW,[[0],[0],[0]]),axis=0)
        self.b = self.dW
        self.dist = norm
    
    def rho0(self,v):
        self.update(v)
        rho0W = super().rho0(v)
        perp = np.cross(np.ravel(self.cW),np.ravel(np.array([[0,0,1]])))
        #TODO: rho_tilde if 0 sign should be + not 0
        rho_tilde = np.copysign(perp,np.dot(np.ravel(rho0W),perp))
        return max((self.zeta-(self.dist-self.R))/(self.zeta - self.delta),0) * (self.cW + np.matrix(rho_tilde).T)


class CylinderConstraint():
    
    def __init__(self,p0,R,delta,zeta,k):
        self.delta = delta
        self.zeta = zeta
        self.p0 = p0
        self.R = R
        self.k = k
    
    def rho0(self,x):
        dist = self.p0 - x[:2]
        #k_rep*max((zeta-np.linalg.norm(x[0:2])**2+epsilon**2)/(zeta-delta),0)*np.concatenate((x[0:2],[[0]]),axis=0)/epsilon
        return self.k*max((self.zeta-(dist-self.R))/(self.zeta - self.delta),0)*np.concatenate((x[0:2],[[0]]),axis=0)/self.R
    
    def Lyapunov_threshold(self,P,v):
        v[2] = 0
        xv = np.concatenate((v,[[0],[0],[0]]),axis=0)
        return xv.T@P@xv

class ERG():
    
    def __init__(self,state_space,P,constraints,k,eta):
        """
        parameters:
            state_space - state space matrices of closed loop system
            P - matrix of the quadratic Lyapunov
            constraints - list of constraints
            k - gain
            eta - smoothing radius
        """
        self.k = k
        self.eta = eta
        
        self.sys = StateSpace(*state_space)
        self.P = P
    
        self.constraints = constraints
        
    def Lyapunov(self,x,v):
        #X = x + self.sys.A.I@self.sys.B@v
        xv = np.concatenate((v,[[0],[0],[0]]),axis=0)
        X = x-xv
        V = X.T@self.P@X
        return V
    
    def Lyapunov_threshold(self,v):
        xv = np.concatenate((v,[[0],[0],[0]]),axis=0)
        return ((self.a.T@xv + self.b)**2)/(self.a.T@self.P.I@self.a)
    
    def compute_reference(self,x,r,v,dt):
        """
        inputs:
            x - current state
            r - true reference
            v - current auxiliary reference
            dt - time step
        outputs:
            v - updated auxiliary reference
        """
        
        #vdot = self.compute_Delta(x,v)*self.compute_attraction_field(r,v)
        
        V = self.Lyapunov(x,v)
        Gamma = self.compute_Gamma(x,v)
        Delta = self.k*(Gamma-V)[0,0]
        rho = self.compute_attraction_field(r,v)
        vdot = Delta*rho
        
        return v + vdot*dt
    
    def compute_attraction_field(self,r,v):
        diff = r-v
        rho = diff/max(np.linalg.norm(diff),self.eta)
        for c in self.constraints:
            rho += c.rho0(v)
        return rho
    
    def compute_Delta(self,x,v):
        V = self.Lyapunov(x,v)
        Delta = None
        for c in self.constraints:
            Gamma = c.Lyapunov_threshold(self.P,v)
            if Delta == None:
                Delta = self.k*(Gamma-V)[0,0]
            else:
                Delta = min(self.k*(Gamma-V)[0,0],Delta)
        return Delta
    
    def compute_Gamma(self,x,v):
        Gamma = None
        for c in self.constraints:
            G = c.Lyapunov_threshold(self.P,v)
            if Gamma == None:
                Gamma = G
            else:
                Gamma = min(G,Gamma)
        return Gamma