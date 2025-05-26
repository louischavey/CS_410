import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import time

def main():
    axis = 'pitch'
    filename = f'data.csv'
    df = pd.read_csv(filename)
    df = df.dropna()
    # breakpoint()
    cam_yaw = np.array(df.iloc[:,0])
    yaw_vel_desired = np.array(df.iloc[:,1])
    yaw_vel_actual = np.array(df.iloc[:,2])
    # roll_desired = np.array(df.iloc[:,3])
    # motor_1 = np.array(df.iloc[:,4])
    # motor_2 = np.array(df.iloc[:,5])
    # motor_3 = np.array(df.iloc[:,6])
    # motor_4 = np.array(df.iloc[:,7])


    t = np.linspace(0, len(cam_yaw)-1, len(cam_yaw))

    

    plt.figure(figsize=(10, 5))
    plt.plot(t, cam_yaw, color='blue', label=f'cam_yaw')
    plt.plot(t, yaw_vel_desired, color='red', label=f'yaw_vel_desired')
    plt.plot(t, yaw_vel_actual, color='green', label=f'yaw_vel_actual')
    # plt.plot(t, roll_filter, color='cyan', label=f'roll_filter')
    # plt.plot(t, motor_1, color='purple', label=f'motor_1')
    # plt.plot(t, motor_2, color='yellow', label=f'motor_2')
    # plt.plot(t, motor_3, color='orange', label=f'motor_3')
    # plt.plot(t, motor_4, color='pink', label=f'motor_4')
    plt.legend()
    plt.savefig(f'auto_yaw.png')

    return

if __name__ == '__main__':
    main()