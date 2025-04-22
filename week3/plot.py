import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

def main():
    axis = 'pitch'
    filename = f'data.csv'
    df = pd.read_csv(filename)
    df = df.dropna()
    # breakpoint()
    pitchx10 = np.array(df.iloc[:,0])
    desired_pitchx10 = np.array(df.iloc[:,1])
    thrust = np.array(df.iloc[:,2])
    motor_front = np.array(df.iloc[:,3])
    motor_back = np.array(df.iloc[:,4])
    t = np.linspace(0, len(pitchx10)-1, len(pitchx10))

    

    plt.figure(figsize=(10, 5))
    plt.plot(t, pitchx10, color='blue', label=f'pitch')
    plt.plot(t, desired_pitchx10, color='orange', label=f'desired Pitch')
    plt.plot(t, thrust, color='green', label=f'Thrust')
    plt.plot(t, motor_front, color='cyan', label=f'Motor_front')
    plt.plot(t, motor_back, color='purple', label=f'Motor_back')
    plt.legend()
    plt.savefig(f'pitch_PID.png')

    return

if __name__ == '__main__':
    main()