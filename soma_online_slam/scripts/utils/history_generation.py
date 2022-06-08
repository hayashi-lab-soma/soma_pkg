import os
from fastslam import History


DIR = os.path.dirname(__file__)
RAW_HISTORY_FILE_NAME = DIR + '/../../data/raw_history.txt'
HISTORY_FILE_NAME = DIR + '/../../data/history.txt'


fo1 = open(RAW_HISTORY_FILE_NAME, "r")
fo2 = open(HISTORY_FILE_NAME, "w")

l = ""
while l[:14] != "particles_num:":
    l = fo1.readline()
    fo2.write(l)

particles_num = int(l[14:])
history = History(particles_num)

while l[:7] != "HISTORY":
    l = fo1.readline()
    fo2.write(l)

fo1.readline()
l = fo1.readline().split(" ")
if len(l) > 1:
    step = l[1][:-1]

while len(l) > 1:
    new_particles_set = []
    for i in range(particles_num):
        l = fo1.readline().split(" ")
        p = []
        p.append(int(l[0][1]))
        p.append(float(l[1][:-1]))
        p.append(float(l[2][:-1]))
        if l[3][-1] == "\n":
            p.append(float(l[3][:-2]))
        else:
            p.append(float(l[3][:-1]))
        new_particles_set.append(p[:])
    history.add_generation(new_particles_set[:], step)

    fo1.readline()
    l = fo1.readline().split(" ")
    if len(l) > 1:
        step = l[1][:-1]

history.stop()
fo2.write(str(history))

fo1.close()
fo2.close()

history.display()
