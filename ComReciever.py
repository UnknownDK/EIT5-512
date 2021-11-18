import serial
import time
from datetime import datetime
import csv

ser = serial.Serial('COM6', 115200, timeout=1)
times = [] #Har byttet rundt på navnene på dem her lol
data = []

input("Press Enter to continue...")
startTime = time.time_ns()
while True:
  try:
    memes = ser.readline().strip().decode('utf-8')
    currTime = time.time_ns() - startTime
    times.append(memes)
    data.append(currTime)
    print(str(currTime) + " " + str(memes))
  except KeyboardInterrupt:
    break


filename = str(datetime.now().strftime("%d%m%Y-%H%M%S")) + ".csv"

with open(filename, 'w', newline='') as f:
    writer = csv.writer(f)
    writer.writerows(zip(data, times))
