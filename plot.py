import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

def main():
    filename = 'data.csv'
    df = pd.read_csv(filename)

    plt.figure(figsize=(10, 5))
    plt.plot(df['x'], df['y'], marker='o', linestyle='-', color='b')


    return

if __name__ == '__main__':
    main()