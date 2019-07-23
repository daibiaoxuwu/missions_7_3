import numpy as np
import math
import vrep
import time

tstep = 0.005             # 定義模擬步長
# 配置關節資訊
print('Program started')
# 關閉潛在的連線
vrep.simxFinish(-1)
# 每隔0.2s檢測一次，直到連線上V-rep
while True:
    clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 20000, 5)
    if clientID > -1:
        break
    else:
        print("Failed connecting to remote API server!")
print("Connection success!")

# 設定模擬步長，為了保持API端與V-rep端相同步長
vrep.simxSetFloatingParameter(clientID, vrep.sim_floatparam_simulation_time_step, tstep, vrep.simx_opmode_oneshot)
# 然後開啟同步模式
vrep.simxSynchronous(clientID, True) 
vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)

# 然後讀取Base和Joint的控制代碼
_,quadricopter_target_handle  = vrep.simxGetObjectHandle(clientID, "Quadricopter_target", vrep.simx_opmode_blocking)
_,quadricopter_handle  = vrep.simxGetObjectHandle(clientID, "Quadricopter_base", vrep.simx_opmode_blocking)
_,v1=vrep.simxGetObjectHandle(clientID, "zed_vision0",  vrep.simx_opmode_blocking)

print('Handles available!')

_, jpos = vrep.simxGetObjectPosition(clientID, quadricopter_target_handle, -1, vrep.simx_opmode_streaming)
_, _, imageBuffer = vrep.simxGetVisionSensorImage(clientID, v1, -1, vrep.simx_opmode_streaming)

lastCmdTime=vrep.simxGetLastCmdTime(clientID)  # 記錄當前時間
vrep.simxSynchronousTrigger(clientID)  # 讓模擬走一步
# 開始模擬

totx = 0
toty = 0
countxy = 0

while vrep.simxGetConnectionId(clientID) != -1:
    currCmdTime=vrep.simxGetLastCmdTime(clientID)  # 記錄當前時間
    dt = currCmdTime - lastCmdTime # 記錄時間間隔，用於控制

    #-------------------------------------------
    _, jpos = vrep.simxGetObjectPosition(clientID, quadricopter_target_handle, quadricopter_handle, vrep.simx_opmode_streaming)
    print(jpos)
    _, shape, imageBuffer = vrep.simxGetVisionSensorImage(clientID, v1, -1, vrep.simx_opmode_streaming)
    maxx=0
    minx=100000
    maxy=0
    miny=100000
    maxd=0
    xlen = 1280
    ylen = 720
    ylock = 0
    out = {}
    for i in range(1,xlen,2):
        maxy2=0
        miny2=100000
        for j in range(0,ylen-30,30):
            if (imageBuffer[i*ylen+j]>0.9 and imageBuffer[i*ylen+j+1]>0.9 and imageBuffer[i*ylen+j+2]>0.9):
                maxx=max(maxx,i)
                minx=min(minx,i)
                maxy2=max(maxy2,j)
                miny2=min(miny2,j)
        if(maxy2 - miny2 < 10000 and maxy2 - miny2 > maxd):
            maxd = maxy2 - miny2;
            maxy = maxy2;
            miny = miny2;

    if not(maxx == 0 or minx == 100000 or maxy == 0 or miny == 100000):
        deltax = (maxx + minx)/2/1280 - 0.5;
        deltay = (maxy + miny)/2/720 - 0.5;
        totx += deltax;
        toty += deltay;
        countxy += 1;
        

        if max(jpos[0],jpos[1],jpos[2])<0.05 and min(jpos[0],jpos[1],jpos[2])>-0.05:
            # if jpos[0]!=0:
            print(deltax,deltay)
            jpos[0]=0.2*(totx / countxy);
            jpos[1]=0.2*(toty / countxy);
            totx = 0;
            toty = 0
            countxy = 0
            jpos[2]=0
            # 控制命令需要同時方式，故暫停通訊，用於儲存所有控制命令一起傳送
            vrep.simxPauseCommunication(clientID, True)
            vrep.simxSetObjectPosition(clientID, quadricopter_target_handle, quadricopter_handle , jpos,  vrep.simx_opmode_oneshot)
            vrep.simxPauseCommunication(clientID, False)    

        #-------------------------------------------


    lastCmdTime=currCmdTime    # 記錄當前時間
    vrep.simxSynchronousTrigger(clientID)  # 進行下一步
    vrep.simxGetPingTime(clientID)    # 使得該模擬步走完
