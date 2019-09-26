import time
import serial
import sys
import numpy as np
import matplotlib.pyplot as plt
from scipy.constants import g

ser = serial.Serial('COM6', 9600)

class calibrate():
    def __init__(self, cell):
        ######################################
        """
        2 = FP
        3 = ED
        5 = IO
        """
        ######################################
        self.n_cell = [4, 5, 2, 3]
        self.index = self.n_cell.index(cell)
        self.V0 = [235.41940255774003, 244.4916253101737, 244.32871416392908, 204.38865312652007]
        self.Vmax = [884.0096605742118, 874.6029776674937, 960.7371453747471, 882.1855386923655]
        self.ticks = []

        self.forceIO = []
        self.forceFP = []
        self.forceED = []

        self.wmax = 4.539

    def measure(self):
        start_time = time.time()
        while time.time() - start_time < 10:
            value = ser.readline()
            split = value.split(b'\t')
            if (len(split) == 3):
                self.IO.append((int(split[0]) - self.V0[1])/(self.Vmax[1] - self.V0[1])*self.wmax)
                self.FP.append((int(split[1]) - self.V0[2])/(self.Vmax[2] - self.V0[2])*self.wmax)
                self.ED.append((int(split[2]) - self.V0[3])/(self.Vmax[3] - self.V0[3])*self.wmax)
                self.ticks.append(time.time() - start_time)
            #current = [int(s) for s in value.split() if s.isdigit()]
            #w += current[0]
            #w += (current[0] - self.V0[self.index])/(self.Vmax[self.index] - self.V0[self.index]) * 4.539
            #count += 1
        #w /= count
        #print(w)


    def plots(self):
        plt.figure()
        plt.plot(self.ticks, self.FP)
        plt.plot(self.ticks, self.ED)
        plt.plot(self.ticks, self.IO)
        plt.legend(('FP', 'ED', 'IO'))
        plt.show()



if __name__ == '__main__':
    cal = calibrate(int(sys.argv[1]))
    cal.measure()
    cal.plots()
