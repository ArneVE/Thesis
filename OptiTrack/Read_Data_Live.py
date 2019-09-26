import natnetclient as natnet
import matplotlib.pyplot as plt
import os
import csv
import time

client = natnet.NatClient(client_ip='157.193.1.33', data_port=1511, comm_port=1510)

DP = client.rigid_bodies['DP'] # Assuming a Motive Rigid Body is available that you named "DP"
MP = client.rigid_bodies['MP']
PP = client.rigid_bodies['PP']
MC = client.rigid_bodies['MC']

markersDP = DP.markers  # returns a list of markers, each with their own properties
markersMP = MP.markers
markersPP = PP.markers
markersMC = MC.markers

flag = 0
data = []
time_vect = []
start_time = time.time()
inter = start_time
print(DP.position)
print(MP.position)
print(PP.position)
print(MC.position)
print(markersMC[0].position[2])
print(DP.position[2])

while (time.time() - start_time < 20):
    if (flag == 0 and DP.position[2] < markersMC[0].position[2]): ## if rigid body DP is higher than the lowest MC marker
        print("Start.")
        start_time = time.time()
        inter = start_time
        flag = 1
    elif (flag == 1):
        if ((time.time() - inter > 0.01)):
            inter = time.time()
            data.append(DP.position[2]) ## [0] = x, [1] = y, [2] = z
            time_vect.append(time.time() - start_time)

#plots = zip(time_vect, data)
#
# with open(raw_data, "w") as f:
#     writer = csv.writer(f)
#     for row in plots:
#         writer.writerow(row)

plt.plot(time_vect, data)
plt.title("Movement Distal Phalanx (keystroke)")
#plt.title("Movement Distal Phalanx")
plt.xlabel("Time [s]")
plt.ylabel("Displacement [m]")
plt.show()
