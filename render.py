import numpy as np
import pygame as pg
import subprocess

result = subprocess.run(['main.exe'], capture_output=True, text=True, check=True)
output = result.stdout.split('\n')[:-1]

tVals = []
xVals1 = []
yVals1 = []
xVals2 = []
yVals2 = []
xVals3 = []
yVals3 = []
xVals4 = []
yVals4 = []
r1 = 0
r2 = 0

for line in output:
    line = line.strip()
    nums = list(map(float, line.split(',')))
    tVals.append(nums[0]) if nums[0] not in tVals else None
    
    if int(nums[1]) == 1:
        xVals1.append(nums[2])
        yVals1.append(nums[3])
        r1 = nums[4]
    elif int(nums[1]) == 2:
        xVals2.append(nums[2])
        yVals2.append(nums[3])
        r2 = nums[4]
    elif int(nums[1]) == 3:
        xVals3.append(nums[2])
        yVals3.append(nums[3])
        r3 = nums[4]
    elif int(nums[1]) == 4:
        xVals4.append(nums[2])
        yVals4.append(nums[3])
        r4 = nums[4]
            
            
print(len(xVals1))
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
        pg.draw.circle(scr, "#ff5a5a", (xVals1[i]*50 + 400, yVals1[i]*50 + 300), r1*50)
        pg.draw.circle(scr, "#4495ff", (xVals2[i]*50 + 400, yVals2[i]*50 + 300), r2*50)
        pg.draw.circle(scr, "#44ff7c", (xVals3[i]*50 + 400, yVals3[i]*50 + 300), r3*50)
        pg.draw.circle(scr, "#ffb744", (xVals4[i]*50 + 400, yVals4[i]*50 + 300), r4*50)
    except:
        t = 0
    
    dt = clock.tick(60)
    t += dt/1000
    i = int(np.floor(t/tMax*(len(tVals))))
    
    pg.display.flip()
    
    