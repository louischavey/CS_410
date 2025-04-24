import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

def main():
    axis = 'pitch'
    filename = f'data.csv'
    df = pd.read_csv(filename)
    df = df.dropna()
    # breakpoint()
    pitchx10 = np.array(df.iloc[:,0]) * 10
    desired_pitchx10 = np.array(df.iloc[:,1])
    # thrust = np.array(df.iloc[:,2])
    motor_1 = np.array(df.iloc[:,2])
    motor_2 = np.array(df.iloc[:,3])
    motor_3 = np.array(df.iloc[:,4])
    motor_4 = np.array(df.iloc[:,5])

    t = np.linspace(0, len(pitchx10)-1, len(pitchx10))

    

    plt.figure(figsize=(10, 5))
    plt.plot(t, pitchx10, color='blue', label=f'pitch_filter')
    plt.plot(t, desired_pitchx10, color='orange', label=f'gyro')
    plt.plot(t, motor_1, color='green', label=f'motor_1')
    plt.plot(t, motor_2, color='cyan', label=f'motor_2')
    plt.plot(t, motor_3, color='red', label=f'motor_3')
    plt.plot(t, motor_4, color='yellow', label=f'motor_4')
    plt.legend()
    plt.savefig(f'D_motor_test.png')

    return

if __name__ == '__main__':
    main()