import numpy as np
import pygame as pg
import subprocess

result = subprocess.run(['cmake-build-debug/main.exe'], capture_output=True, text=True, check=True)
output = result.stdout.split('\n')[:-1]

tVals = []
posVals1 = []
posVals2 = []
verts1 = [(1,1),(-1,1),(-1,-1),(1,-1)]
verts2 = [(1,1),(-1,1),(-1,-1),(1,-1)]

for line in output:
    line = line.strip()
    print(line)
    nums = list(map(float, line.split(',')))
    tVals.append(nums[0]) if nums[0] not in tVals else None

    if int(nums[1]) == 1:
        posVals1.append((nums[2], nums[3]))
    elif int(nums[1]) == 2:
        posVals2.append((nums[2], nums[3]))
        
    # elif int(nums[1]) == 3:
    #     xVals3.append(nums[2])
    #     yVals3.append(nums[3])
    #     xVals1.append((nums[4], nums[5]))
    # elif int(nums[1]) == 4:
    #     xVals4.append(nums[2])
    #     yVals4.append(nums[3])
    #     xVals1.append((nums[4], nums[5]))

vertsMov1 = []
vertsMov2 = []

for pos1, pos2 in zip(posVals1, posVals2):
    vertsMov1.append([((pos1[0]+verts1[0][0])*100+400, (pos1[1]+verts1[0][1])*100+400),
                      ((pos1[0]+verts1[1][0])*100+400, (pos1[1]+verts1[1][1])*100+400),
                      ((pos1[0]+verts1[2][0])*100+400, (pos1[1]+verts1[2][1])*100+400),
                      ((pos1[0]+verts1[3][0])*100+400, (pos1[1]+verts1[3][1])*100+400)])
    
    vertsMov2.append([((pos2[0]+verts2[0][0])*100+400, (pos2[1]+verts2[0][1])*100+400),
                      ((pos2[0]+verts2[1][0])*100+400, (pos2[1]+verts2[1][1])*100+400),
                      ((pos2[0]+verts2[2][0])*100+400, (pos2[1]+verts2[2][1])*100+400),
                      ((pos2[0]+verts2[3][0])*100+400, (pos2[1]+verts2[3][1])*100+400)])
    

    
pg.init()
scr = pg.display.set_mode((800, 600))
clock = pg.time.Clock()
running = True
i = 0
tMax = tVals[-1]
t = 0

while running:
    for event in pg.event.get():
        if event.type == pg.QUIT:
            running = False

    scr.fill("#e7dcdc")

    try:
        pg.draw.polygon(scr, 'red', vertsMov1[i])
        pg.draw.polygon(scr, 'blue', vertsMov2[i])

    except:
        t = 0

    dt = clock.tick(60)
    t += dt/1000
    i = int(np.floor(t/tMax*(len(tVals))))

    pg.display.flip()