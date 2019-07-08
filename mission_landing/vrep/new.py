import numpy as np
import math
import vrep

RAD2EDG = 180 / math.pi   # 常數，弧度轉度數
tstep = 0.005             # 定義模擬步長
# 配置關節資訊
jointNum = 6
baseName = 'Jaco'
jointName = 'Jaco_joint'
print('Program started')
# 關閉潛在的連線
vrep.simxFinish(-1)
# 每隔0.2s檢測一次，直到連線上V-rep
while True:
    clientID = vrep.simxStart('127.0.0.1', 19999, True, True, 5000, 5)
    if clientID > -1:
        break
    else:
        time.sleep(0.2)
        print("Failed connecting to remote API server!")
print("Connection success!")

# 設定模擬步長，為了保持API端與V-rep端相同步長
vrep.simxSetFloatingParameter(clientID, vrep.sim_floatparam_simulation_time_step, tstep, vrep.simx_opmode_oneshot)
# 然後開啟同步模式
vrep.simxSynchronous(clientID, True) 
vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)

# 然後讀取Base和Joint的控制代碼
jointHandle = np.zeros((jointNum,), dtype=np.int) # 注意是整型
for i in range(jointNum):
    _, returnHandle = vrep.simxGetObjectHandle(clientID, jointName + str(i+1), vrep.simx_opmode_blocking)
    jointHandle[i] = returnHandle

_, baseHandle = vrep.simxGetObjectHandle(clientID, baseName, vrep.simx_opmode_blocking)

print('Handles available!')

# 然後首次讀取關節的初始值，以streaming的形式
jointConfig = np.zeros((jointNum,))
for i in range(jointNum):
     _, jpos = vrep.simxGetJointPosition(clientID, jointHandle[i], vrep.simx_opmode_streaming)
     jointConfig[i] = jpos

    lastCmdTime=vrep.simxGetLastCmdTime(clientID)  # 記錄當前時間
    vrep.simxSynchronousTrigger(clientID)  # 讓模擬走一步
# 開始模擬
    while vrep.simxGetConnectionId(clientID) != -1：
        currCmdTime=vrep.simxGetLastCmdTime(clientID)  # 記錄當前時間
        dt = currCmdTime - lastCmdTime # 記錄時間間隔，用於控制

        #-------------------------------------------
        # 讀取當前的狀態值，之後都用buffer形式讀取
        for i in range(jointNum):
            _, jpos = vrep.simxGetJointPosition(clientID, jointHandle[i], vrep.simx_opmode_buffer)
            print(round(jpos * RAD2DEG, 2))
            jointConfig[i] = jpos

        # 控制命令需要同時方式，故暫停通訊，用於儲存所有控制命令一起傳送
        vrep.simxPauseCommunication(clientID, True)
        for i in range(jointNum):
            vrep.simxSetJointTargetPosition(clientID, jointHandle[i], 120/RAD2DEG, vrep.simx_opmode_oneshot)
        vrep.simxPauseCommunication(clientID, False)    
        #-------------------------------------------


        lastCmdTime=currCmdTime    # 記錄當前時間
        vrep.simxSynchronousTrigger(clientID)  # 進行下一步
        vrep.simxGetPingTime(clientID)    # 使得該模擬步走完
