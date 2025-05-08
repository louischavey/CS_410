import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

def main():
    axis = 'pitch'
    filename = f'data.csv'
    df = pd.read_csv(filename)
    df = df.dropna()
    # breakpoint()
    roll_desired = np.array(df.iloc[:,0]) * 10
    roll_filter = np.array(df.iloc[:,1]) * 10
    pitch_desired = np.array(df.iloc[:,2]) * 10
    pitch_filter = np.array(df.iloc[:,3]) * 10
    motor_1 = np.array(df.iloc[:,4]) / 10
    motor_2 = np.array(df.iloc[:,5]) / 10
    motor_3 = np.array(df.iloc[:,6]) / 10
    motor_4 = np.array(df.iloc[:,7]) / 10
    thrust = np.array(df.iloc[:,8]) / 10

    t = np.linspace(0, len(roll_desired)-1, len(roll_desired))
    

    plt.figure(figsize=(10, 5))
    plt.plot(t, roll_desired, color='blue', label=f'roll_desired')
    plt.plot(t, roll_filter, color='red', label=f'roll_filter')
    plt.plot(t, pitch_desired, color='orange', label=f'pitch_desired')
    plt.plot(t, pitch_filter, color='black', label=f'pitch_filter')
    plt.plot(t, motor_1, color='green', label=f'front_left')
    plt.plot(t, motor_2, color='cyan', label=f'back_left')
    plt.plot(t, motor_3, color='purple', label=f'front_right')
    plt.plot(t, motor_4, color='yellow', label=f'back_right')
    plt.plot(t, thrust, color='brown', label=f'thrust')
    plt.legend()
    plt.savefig(f'No_Rig_Test.png')

    return

if __name__ == '__main__':
    main()