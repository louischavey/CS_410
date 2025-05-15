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
    pitch_filter = np.array(df.iloc[:,0])
    pitch_desired = np.array(df.iloc[:,1])
    roll_filter = np.array(df.iloc[:,2])
    roll_desired = np.array(df.iloc[:,3])
    motor_1 = np.array(df.iloc[:,4])
    motor_2 = np.array(df.iloc[:,5])
    motor_3 = np.array(df.iloc[:,6])
    motor_4 = np.array(df.iloc[:,7])


    t = np.linspace(0, len(roll_desired)-1, len(roll_desired))

    

    plt.figure(figsize=(10, 5))
    plt.plot(t, pitch_desired, color='blue', label=f'pitch_desired')
    plt.plot(t, pitch_filter, color='red', label=f'pitch_filter')
    plt.plot(t, roll_desired, color='green', label=f'roll_desired')
    plt.plot(t, roll_filter, color='cyan', label=f'roll_filter')
    # plt.plot(t, motor_1, color='purple', label=f'motor_1')
    # plt.plot(t, motor_2, color='yellow', label=f'motor_2')
    # plt.plot(t, motor_3, color='orange', label=f'motor_3')
    # plt.plot(t, motor_4, color='pink', label=f'motor_4')
    plt.legend()
    plt.savefig(f'drift_test.png')

    return

if __name__ == '__main__':
    main()