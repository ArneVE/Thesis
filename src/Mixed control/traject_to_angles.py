import numpy as np
import pdb
from scipy.optimize import fsolve
from mpmath import sin, cos, radians, degrees
from math import pi

class traject_to_angles():
    def __init__(self, x, z):
        ############################
        """
        S1, S2, S3 measured with OptiTrack

        Values subtracted from x (0.063141) and z (0.056435), because the origin
        of the trajectory is at the Metacarpal, but for the inverse kinematics
        it is at the MCP joint!! Also measured with OptiTrack!
        """
        ############################
        self.S1 = 0.099546229
        self.S2 = 0.066872175
        self.S3 = 0.042911243

        self.x_vect = x - 0.063141
        self.z_vect = z - 0.056435

        # MCP = MCP angle
        # PIP = PIP angle
        # DIP = DIP angle
        self.MCP_deg = []
        self.PIP_deg = []
        self.DIP_deg = []

        self.MCP_rad = []
        self.PIP_rad = []
        self.DIP_rad = []


    def equations(self, p):
        MCP, PIP = p

        DIP = radians(-6E-11*(degrees(PIP))**6 + 8E-09*(degrees(PIP))**5 - 3E-08*(degrees(PIP))**4 + 2E-05*(degrees(PIP))**3 + 0.0014*(degrees(PIP))**2 + 0.3904*(degrees(PIP)) - 0.7545)
        return (self.S1*cos(MCP) + self.S2*cos(MCP + PIP) + self.S3*cos(MCP + PIP + DIP) - self.x, self.S1*sin(MCP) + self.S2*sin(MCP + PIP) + self.S3*sin(MCP + PIP + DIP) - self.z)


    def run(self):
        self.i = 0
        for self.i in range(len(self.x_vect)):
            self.x = self.x_vect[self.i]
            self.z = self.z_vect[self.i]
            if (self.i == 0):
                MCP, PIP = fsolve(self.equations, (-0.2, 0.2))
            else:
                MCP, PIP = fsolve(self.equations, (self.MCP_rad[self.i - 1], self.PIP_rad[self.i - 1]))

            DIP = radians(-6E-11*(degrees(PIP))**6 + 8E-09*(degrees(PIP))**5 - 3E-08*(degrees(PIP))**4 + 2E-05*(degrees(PIP))**3 + 0.0014*(degrees(PIP))**2 + 0.3904*(degrees(PIP)) - 0.7545)

            while MCP < -pi:
                MCP += 2*pi
            while MCP > pi:
                MCP -= 2*pi

            self.MCP_deg.append(degrees(MCP))
            self.PIP_deg.append(degrees(PIP))
            self.DIP_deg.append(degrees(DIP))

            self.MCP_rad.append((MCP))
            self.PIP_rad.append((PIP))
            self.DIP_rad.append((DIP))

        # for i in range(len(self.MCP_deg)):
        #     print("(", self.MCP_deg[i], ", ", self.PIP_deg[i], ", ", self.DIP_deg[i], ")")


if __name__ == '__main__':
    x_vect = [5.5 for i in np.arange(0, 2, 0.1)]
    z_vect = [i for i in np.arange(0, 2, 0.1)]

    sol = traject_to_angles(x_vect, z_vect)
    sol.run()
