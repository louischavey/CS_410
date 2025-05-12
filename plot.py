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
    plt.plot(t, roll_desired, color='blue', label=f'roll_desired')
    plt.plot(t, roll_filter, color='red', label=f'roll_filter')
    plt.plot(t, motor_1, color='green', label=f'motor_1')
    plt.plot(t, motor_2, color='cyan', label=f'motor_2')
    plt.plot(t, motor_3, color='purple', label=f'motor_3')
    plt.plot(t, motor_4, color='yellow', label=f'motor_4')
    plt.legend()
    plt.savefig(f'Roll_PID.png')

    return

if __name__ == '__main__':
    main()