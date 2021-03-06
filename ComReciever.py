import serial
from datetime import datetime
import csv
import matplotlib.pyplot as plt

ser = serial.Serial('COM6', 115200, timeout=1)
#times = [] #Har byttet rundt på navnene på dem her lol
data = []
num = []
times = []
memes = []
input("Press Enter to continue...")
while True:
  try:
    memes.append(ser.readline())
  except KeyboardInterrupt:
    break

for i in memes:
  #print(i)
  temp = i.strip().decode('utf-8').split(",")
  if len(temp) == 2:
    num.append(temp[0])
    times.append(temp[1])
filename = str(datetime.now().strftime("%d%m%Y-%H%M%S")) + ".csv"

with open(filename, 'w', newline='') as f:
  writer = csv.writer(f)
  writer.writerows(zip(times,num))

for i in range(len(times)):
  times[i] = int(times[i])/1000


plt.plot(times, num)
plt.show()
