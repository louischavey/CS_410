import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

def main():
    axis = 'pitch'
    filename = f'data.csv'
    df = pd.read_csv(filename)
    df = df.dropna()
    # breakpoint()
    pitch_desired = np.array(df.iloc[:,0]) * 10
    pitch_filter = np.array(df.iloc[:,1]) * 10
    motor_1 = np.array(df.iloc[:,2]) / 10
    motor_2 = np.array(df.iloc[:,3]) / 10
    motor_3 = np.array(df.iloc[:,4]) / 10
    motor_4 = np.array(df.iloc[:,5]) / 10

    t = np.linspace(0, len(pitch_desired)-1, len(pitch_desired))

    

    plt.figure(figsize=(10, 5))
    plt.plot(t, pitch_desired, color='blue', label=f'pitch_desired')
    plt.plot(t, pitch_filter, color='red', label=f'pitch_filter')
    plt.plot(t, motor_1, color='green', label=f'motor_1')
    plt.plot(t, motor_2, color='cyan', label=f'motor_2')
    plt.plot(t, motor_3, color='purple', label=f'motor_3')
    plt.plot(t, motor_4, color='yellow', label=f'motor_4')
    plt.legend()
    plt.savefig(f'test.png')

    return

if __name__ == '__main__':
    main()