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
    pitch_desired = np.array(df.iloc[:,0])
    pitch_filter = np.array(df.iloc[:,1])
    roll_desired = np.array(df.iloc[:,2])
    roll_filter = np.array(df.iloc[:,3])
    yaw_desired = np.array(df.iloc[:,4])
    yaw_raw = np.array(df.iloc[:,5])

    t = np.linspace(0, len(roll_desired)-1, len(roll_desired))

    

    plt.figure(figsize=(10, 5))
    plt.plot(t, pitch_desired, color='blue', label=f'motor_1')
    plt.plot(t, pitch_filter, color='red', label=f'motor_2')
    plt.plot(t, roll_desired, color='green', label=f'motor_3')
    plt.plot(t, roll_filter, color='cyan', label=f'motor_4')
    plt.plot(t, yaw_desired, color='purple', label=f'yaw_desired')
    plt.plot(t, yaw_raw, color='yellow', label=f'yaw_raw')
    plt.legend()
    plt.savefig(f'yaw.png')

    return

if __name__ == '__main__':
    main()