import time
import serial
import sys
import numpy as np
import matplotlib.pyplot as plt
from scipy.constants import g

ser = serial.Serial('COM6', 9600)

class calibrate():
    def __init__(self, V, cell):
        ######################################
        # 4 = FP
        # 5 = ED
        # 2 = IO
        ######################################
        self.n_cell = [4, 5, 2, 3]
        self.V = V
        self.index = self.n_cell.index(cell)
        self.V0 = [235.41940255774003, 244.4916253101737, 244.32871416392908, 204.38865312652007]
        self.Vmax = [884.0096605742118, 874.6029776674937, 960.7371453747471, 882.1855386923655]
        self.ticks = []
        self.data = []

    def measure(self):
        start_time = time.time()
        count = 0
        w = 0
        if self.V == 'V0':
            while time.time() - start_time < 10:
                value = ser.readline()
                current = [int(s) for s in value.split() if s.isdigit()]
                self.V0[self.index] += current[0]
                count += 1
                self.data.append(current[0])
                self.ticks.append(time.time() - start_time)
            self.V0[self.index] /= count
            print(self.V0[self.index])
            self.plots()

        elif self.V == 'Vmax':
            while time.time() - start_time < 10:
                value = ser.readline()
                current = [int(s) for s in value.split() if s.isdigit()]
                self.Vmax[self.index] += current[0]
                count += 1
                self.data.append(current[0])
                self.ticks.append(time.time() - start_time)
            self.Vmax[self.index] /= count
            print(self.Vmax[self.index])
            self.plots()

        elif self.V == 'test':
            while time.time() - start_time < 10:
                value = ser.readline()
                current = [int(s) for s in value.split() if s.isdigit()]
                #w += current[0]
                w += (current[0] - self.V0[self.index])/(self.Vmax[self.index] - self.V0[self.index]) * 4.539
                count += 1
            w /= count
            print(w)

        else:
            print("Wrong input")


    def plots(self):
        plt.figure()
        plt.plot(self.ticks, self.data)
        plt.show()


    def solver(self):
        a = np.array([[self.V0[self.index],1],[self.Vmax[self.index],1]])
        b = np.array([0, g*4.539])
        print(np.linalg.solve(a,b))



if __name__ == '__main__':
    cal = calibrate(sys.argv[1], int(sys.argv[2]))
    cal.measure()
    #cal.solver()
