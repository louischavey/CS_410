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
    auto_pitch_desired = np.array(df.iloc[:,1])
    camera_x_estimated = np.array(df.iloc[:,2])
    auto_pitch_desired_p = np.array(df.iloc[:,3])
    auto_pitch_desired_d = np.array(df.iloc[:,4])
    x_desired = np.array(df.iloc[:,5])
    roll_filter = np.array(df.iloc[:,6])
    auto_roll_desired = np.array(df.iloc[:,7])
    camera_y_estimated = np.array(df.iloc[:,8])
    auto_roll_desired_p = np.array(df.iloc[:,9])
    auto_roll_desired_d = np.array(df.iloc[:,10])
    y_desired = np.array(df.iloc[:,11])


    t = np.linspace(0, len(pitch_filter)-1, len(pitch_filter))

    

    plt.figure(figsize=(10, 5))
    # plt.plot(t, pitch_filter, color='blue', label=f'pitch_filter')
    # plt.plot(t, auto_pitch_desired, color='red', label=f'auto_pitch_desired')
    plt.plot(t, camera_x_estimated, color='green', label=f'camera_x_estimated')
    # plt.plot(t, auto_pitch_desired_p, color='cyan', label=f'auto_pitch_desired_p')
    # plt.plot(t, auto_pitch_desired_d, color='purple', label=f'auto_pitch_desired_d')
    plt.plot(t, x_desired, color='yellow', label=f'x_desired')
    # plt.plot(t, roll_filter, color='orange', label=f'roll_filter')
    # plt.plot(t, auto_roll_desired, color='magenta', label=f'auto_roll_desired')
    # plt.plot(t, camera_y_estimated, color='brown', label=f'camera_y_estimated')
    # plt.plot(t, auto_roll_desired_p, color='gray', label=f'auto_roll_desired_p')
    # plt.plot(t, auto_roll_desired_d, color='black', label=f'auto_roll_desired_d')
    # plt.plot(t, y_desired, color='turquoise', label=f'y_desired')

    plt.legend()
    plt.savefig(f'Desired and estimated x.png')

    return

if __name__ == '__main__':
    main()