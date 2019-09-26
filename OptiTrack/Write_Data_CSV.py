import natnetclient as natnet
import matplotlib.pyplot as plt
import os
import csv
import time

client = natnet.NatClient(client_ip='172.17.210.208', data_port=1511, comm_port=1510)

DP = client.rigid_bodies['DP'] # Assuming a Motive Rigid Body is available that you named "DP"
MP = client.rigid_bodies['MP']
PP = client.rigid_bodies['PP']
Meta = client.rigid_bodies['Metacarpal']

markersDP = DP.markers  # returns a list of markers, each with their own properties
markersMP = MP.markers
markersPP = PP.markers
markersMeta = Meta.markers

flag = 0
DP_data1x = []
DP_data2x = []
DP_data1z = []
DP_data2z = []
MP_data1x = []
MP_data2x = []
MP_data1z = []
MP_data2z = []
PP_data1x = []
PP_data2x = []
PP_data1z = []
PP_data2z = []
time_vect = []
start_time = time.time()
inter_time = start_time
print(markersDP)
print(markersMP)
print(markersPP)

# Get positions of 2 markers that will define the angle
DP_dist = 1
MP_dist = 1
PP_dist = 1
for i in range(0, 3):
    for j in range(i+1,3):
        if (abs(markersDP[i].position[2] - markersDP[j].position[2]) < DP_dist):
            DP_dist = abs(markersDP[i].position[2] - markersDP[j].position[2])
            DP_index1 = i
            DP_index2 = j
        if (abs(markersMP[i].position[2] - markersMP[j].position[2]) < MP_dist):
            MP_dist = abs(markersMP[i].position[2] - markersMP[j].position[2])
            MP_index1 = i
            MP_index2 = j
        if (abs(markersPP[i].position[2] - markersPP[j].position[2]) < PP_dist):
            PP_dist = abs(markersPP[i].position[2] - markersPP[j].position[2])
            PP_index1 = i
            PP_index2 = j

print(DP_index1)
print(DP_index2)
print(MP_index1)
print(MP_index2)
print(PP_index1)
print(PP_index2)

# Measure the 4 relevant data points
print("Start.")
while (time.time() - start_time < 60):
    # Cameras have frame rate of 120 fps
    if (time.time() - inter_time > 1/120):
        DP_data1z.append(DP.markers[DP_index1].position[2]) ## [0] = x, [1] = y, [2] = z
        DP_data2z.append(DP.markers[DP_index2].position[2])
        DP_data1x.append(DP.markers[DP_index1].position[0])
        DP_data2x.append(DP.markers[DP_index2].position[0])
        MP_data1z.append(MP.markers[MP_index1].position[2])
        MP_data2z.append(MP.markers[MP_index2].position[2])
        MP_data1x.append(MP.markers[MP_index1].position[0])
        MP_data2x.append(MP.markers[MP_index2].position[0])
        PP_data1z.append(PP.markers[PP_index1].position[2])
        PP_data2z.append(PP.markers[PP_index2].position[2])
        PP_data1x.append(PP.markers[PP_index1].position[0])
        PP_data2x.append(PP.markers[PP_index2].position[0])
        time_vect.append(time.time() - start_time)
        inter_time = time.time()

print("Done.")

plots = zip(time_vect, DP_data1x, DP_data1z, DP_data2x, DP_data2z, MP_data1x, MP_data1z, MP_data2x, MP_data2z, PP_data1x, PP_data1z, PP_data2x, PP_data2z)
with open('raw_data.csv', "w") as f:
    writer = csv.writer(f)
    writer.writerow(['Time,DP_data1x,DP_data1z,DP_data2x,DP_data2z,MP_data1x,MP_data1z,MP_data2x,MP_data2z,PP_data1x,PP_data1z,PP_data2x,PP_data2z'])
    for row in plots:
        writer.writerow(row)
