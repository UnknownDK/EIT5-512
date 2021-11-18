import serial
import time
from datetime import datetime
import csv

ser = serial.Serial('COM6', 115200, timeout=1)
startTime = time.time_ns()
times = []
data = []


while True:
  try:
    memes = ser.readline().strip().decode('utf-8')
    currTime = time.time_ns() - startTime
    times.append(memes)
    data.append(currTime/100000)
    print(str(currTime/100000) + " " + str(memes))
  except KeyboardInterrupt:
    break

filename = str(datetime.now().strftime("%d%m%Y-%H%M%S")) + ".csv"

with open(filename, 'w', newline='') as f:
    writer = csv.writer(f)
    writer.writerows(zip(data, times))
