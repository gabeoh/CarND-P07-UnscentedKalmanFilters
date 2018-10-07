import pandas as pd
import matplotlib.pyplot as plt

# Normalized Innovation Squred (NIS) for Lidar and Radar
# Using std_a_ = 3 and std_yawdd_ = 0.75
# RMSE: x=0.0732, y=0.0849, vx=0.3796, vy=0.2388
nis_lidar_str = "0.139201 0.117753 0.257664 0.0321584 0.875967 2.64295 0.280254 11.0243 3.25713 1.92872 3.64574 0.154625 2.48049 0.478255 0.241212 0.667697 0.631512 2.87491 1.06782 1.17437 2.2382 0.24372 0.720391 3.15098 2.4198 0.918854 0.496747 2.51274 0.739676 0.351605 0.314198 0.285651 0.493687 0.665956 0.113492 1.32777 1.07604 0.929567 1.34625 3.56234 2.00568 1.26642 2.99724 3.93124 3.49559 1.27625 1.17202 2.17179 0.0278736 5.10515 0.871052 2.10151 0.885761 1.46167 0.412672 1.02327 2.29788 3.1185 0.22985 0.341673 0.494385 2.09521 3.60742 1.02556 1.05152 0.64224 0.203612 2.78665 0.754415 0.700176 1.0569 1.9925 0.583616 0.686868 1.39182 0.253439 2.20426 0.141774 1.8433 0.601564 0.32478 3.90294 1.36037 0.424002 3.36245 3.086 0.105294 2.58838 0.143527 0.108483 0.239361 2.41025 2.00243 0.649139 1.02531 1.28629 2.91228 1.15807 6.23418 0.0158292 0.504233 2.25519 1.61741 0.5931 1.69597 0.737587 0.546156 4.59702 0.98032 3.84715 0.409001 0.430067 4.39406 2.30654 0.751784 2.18226 0.899207 3.81154 0.0607892 1.12153 0.707609 0.310905 2.25053 0.146233 2.08685 0.730829 2.00528 1.8376 1.01222 0.0202629 1.26133 0.48742 1.1583 5.50923 3.75095 1.55763 1.51396 1.9283 1.89971 1.06877 0.403185 0.520661 2.36253 1.50157 4.75354 0.579611 1.22159 0.29144 1.9369 0.0433497 3.42853 0.931516 0.698749 3.77112 3.42509 3.23115 2.73379 0.42568 1.49566 4.42331 1.60627 1.23935 0.891424 1.70209 5.35458 2.1243 0.518258 1.67717 0.503141 0.205421 1.90438 0.28847 3.12928 1.26505 14.0812 0.842205 1.00412 1.05951 0.0302381 1.64721 6.63196 2.16618 0.816124 2.29259 2.95305 5.22049 2.92613 0.346626 0.581589 0.563302 0.485233 1.55259 4.15237 1.51605 5.54785 0.818871 1.35143 0.0106704 0.810343 2.5026 0.858658 2.41709 0.432174 0.671707 0.749056 0.255954 1.48933 0.327573 3.5862 2.20667 0.3822 1.03053 1.2036 0.0495535 0.0411243 4.62493 0.979291 0.320908 2.11085 0.0394598 4.508 3.11144 1.42618 0.503045 0.543851 6.2752 0.72593 2.42277 0.391227 0.409535 1.12143 1.68128 2.12082 0.184993 2.05297 7.91426 0.586307 3.94215 1.86789 2.09956 0.653699 2.66562 1.07892 2.0936 0.803566 1.17782 3.66784 7.02568 0.72343"
nis_radar_str = "8.20111 1.93811 4.48933 3.4879 0.149959 1.82515 3.82662 1.49657 1.02665 1.87651 3.35487 0.285383 1.78564 5.35872 2.20076 6.55818 6.78762 4.29017 4.64342 2.5881 2.63208 0.629735 1.45978 0.61171 2.45834 10.7596 0.108338 1.34129 3.24834 2.5573 1.13718 1.36452 0.744233 2.9318 1.50859 2.71544 0.499486 0.96542 0.265588 0.777905 3.53882 0.282144 2.10641 0.411742 0.908899 2.10689 7.53596 2.65232 4.83753 9.53302 4.17996 2.79839 2.10536 1.35496 0.755795 1.34294 3.41949 7.43369 1.23235 1.83985 0.205242 6.60223 6.0418 2.05291 2.28516 6.46646 1.82739 3.01113 3.2781 10.1587 3.25172 5.42112 2.23232 1.21687 0.289112 2.09928 1.90826 2.52911 0.93052 1.05241 4.12605 5.57919 1.00761 1.68963 1.43295 4.31953 0.084717 3.37361 0.257303 0.590437 3.04676 1.62862 1.8752 4.60115 1.64388 1.07331 6.98623 3.6375 2.03006 2.41365 2.35886 4.57095 0.559466 2.20814 0.976057 2.41133 1.94492 1.865 0.42466 1.5109 2.77101 3.35049 2.73855 1.98243 1.64106 1.0522 0.655007 2.29828 2.04086 2.46398 5.46581 4.93853 1.87989 2.99264 4.22252 2.15394 0.306889 1.92197 1.61464 4.01442 0.694042 0.258918 3.94868 0.238343 1.14866 5.61101 5.82514 3.19479 5.44477 2.92142 2.76491 3.5652 1.05715 0.366183 4.37705 0.506535 0.411959 1.92534 2.85039 0.280455 4.94642 1.33999 6.98863 0.713112 1.64808 1.41676 1.63873 0.888859 2.86003 1.39967 2.66045 3.15906 2.71584 0.444328 4.49738 1.83759 1.97172 4.77465 7.99428 1.6621 0.499293 1.79595 3.83112 1.10408 4.85837 1.6471 2.60138 1.99734 5.75443 2.36661 0.724796 3.24884 1.85479 0.850752 1.93593 3.47812 0.308823 1.83391 1.29413 6.1642 0.145286 11.3141 3.34133 0.294633 1.59638 1.22794 2.12179 1.54015 5.10626 2.3068 1.94935 1.89644 6.57785 8.63234 0.325082 0.595982 3.01703 2.11295 1.13627 0.561729 8.95871 2.32201 0.820469 3.10407 4.45218 1.44101 5.47327 1.60991 2.2132 5.68215 1.58314 5.3293 4.99179 1.47274 0.791522 4.88288 1.65599 5.23328 3.02954 0.347588 0.0267971 0.404001 0.201886 7.61164 6.38656 0.571856 1.57376 0.318682 1.28017 0.427403 2.7973 8.38359 0.433827 6.15362 3.78321 2.43201 6.24252 1.50969 1.91405"

def draw_plot(nis, chi_sq_95, name, outfile=None):
    df_nis = pd.DataFrame({
        name: nis
    }, index=range(len(nis)), columns=[name])

    count_over_limit = len([i for i in nis if i > chi_sq_95])
    msg = "{} out of {} are over the 95% limit ({:.2%})".format(count_over_limit, len(nis), count_over_limit/len(nis))

    nis_plt = df_nis.plot()
    nis_plt.axhline(y=chi_sq_95, color='r', linestyle='-')
    nis_plt.text(10, max(nis), msg, style='italic', horizontalalignment='left',
                 bbox={'facecolor': 'red', 'alpha': 0.5})
    plt.tight_layout()
    if (outfile):
        plt.savefig(outfile)
    plt.show()

nis_lidar = [float(a) for a in nis_lidar_str.split(" ")]
nis_radar = [float(a) for a in nis_radar_str.split(" ")]

# Chi square 95% for Lidar(df=2) and Radar(df=3) are: 5.991 and 7.815 respectively
draw_plot(nis_lidar, 5.991, 'NIS Lidar', 'NIS_Lidar.png')
draw_plot(nis_radar, 7.815, 'NIS Radar', 'NIS_Radar.png')