import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

def main():
    df = pd.read_csv("ss_data.csv")

    df["x"].fillna(method='ffill', inplace=True)
    df["y"].fillna(method='ffill', inplace=True)
    df["z"].fillna(method='ffill', inplace=True)

    df["vx"].fillna(0, inplace=True)
    df["vy"].fillna(0, inplace=True)
    df["vz"].fillna(0, inplace=True)

    time = df["time"].to_numpy()
    x = df["x"].to_numpy()
    y = df["y"].to_numpy()
    z = df["z"].to_numpy()
    vx = df["vx"].to_numpy()
    vy = df["vy"].to_numpy()
    vz = df["vz"].to_numpy()
    
    velocity_magnitude = np.sqrt(vx**2 + vy**2 + vz**2)

    fig, axs = plt.subplots(5, 2, figsize=(10, 12))

    axs[0, 0].plot(time, x)
    axs[0, 0].set_ylabel("x")

    axs[1, 0].plot(time, y)
    axs[1, 0].set_ylabel("y")

    axs[2, 0].plot(time, z)
    axs[2, 0].set_ylabel("z")

    axs[0, 1].plot(time, vx)
    axs[0, 1].set_ylabel("vx")

    axs[1, 1].plot(time, vy)
    axs[1, 1].set_ylabel("vy")

    axs[2, 1].plot(time, vz)
    axs[2, 1].set_ylabel("vz")

    axs[3, 0].plot(y, z)
    axs[3, 0].set_ylabel("path2D y-z")

    axs[3, 1].plot(x, y)
    axs[3, 1].set_ylabel("path2D x-y")

    axs[4, 0].plot(x, z)
    axs[4, 0].set_ylabel("path2D x-z")

    axs[4, 1].plot(time,velocity_magnitude, color='k')
    axs[4, 1].set_ylabel("Abs Vel")

    for i in range(3):
        for j in range(1):
            axs[i, j].set_xlabel("time")

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
