import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

def main():
    axis = 'pitch'
    filename = f'{axis}_data.csv'
    df = pd.read_csv(filename)
    accel = np.array(df.iloc[:,3])
    intl = np.array(df.iloc[:,4])
    filter = np.array(df.iloc[:,5])
    t = np.linspace(0, len(accel)-1, len(accel))

    

    plt.figure(figsize=(10, 5))
    plt.plot(t, accel, color='blue', label=f'accel_{axis}')
    plt.plot(t, intl, color='red', label=f'intl_{axis}')
    plt.plot(t, filter, color='yellow', label=f'filter_{axis}')
    plt.legend()
    plt.savefig(f'{axis}_plot.png')

    return

if __name__ == '__main__':
    main()