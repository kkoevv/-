import numpy as np
from scipy.integrate import odeint
import matplotlib.pyplot as plt

# Константы и исходные данные
G = 6.67e-11    # гравитационная постоянная
R = 600000.0   # радиус планеты (м)
M = 5.292e22   # масса планеты (кг)
g0 = 9.81       # Ускорение свободного падения у поверхности (примерно)
rho_0 = 1.3     # Плотность у поверхности (кг/м^3)
h_e0 = 7600.0    # Масштаб высоты для плотности
Cx = 0.52       # Коэффициент лобового сопротивления
d = 5.14        # Диаметр ракеты (м)
S = np.pi * (d**2) / 4.0  # Площадь поперечного сечения

# Орбитальная высота и скорость
h_orbit = 80000  # Высота орбиты (м)
v_orbit = np.sqrt(G * M / (R + h_orbit))  # Орбитальная скорость (м/с)

# Масса
start_mass = 20.98 * 1000.0

# Параметры ступеней (из документа)
t0 = 23.7
t1 = 58.4
t2 = 74.3
t3 = 307.7
T = t1 + t2 + t3

omega_booster = 118.71
omega_stage1 = 68.51
omega_stage2 = 17.73
omega_stage3 = 0.65

booster_total = 3.8 * 1000
booster_fuel = 2813
booster_dry = booster_total - booster_fuel

stage1_total = 10.63 * 1000
stage1_fuel = 4000
stage1_dry = stage1_total - stage1_fuel

stage2_total = 1.86 * 1000
stage2_fuel = 1300
stage2_dry = stage2_total - stage2_fuel

stage3_total = 0.7 * 1000
stage3_fuel = 200
stage3_dry = stage3_total - stage3_fuel

def m(t):
    if t < 0:
        return start_mass
    if t <= t0:
        return start_mass - omega_booster * t - omega_stage1 * t
    mass_after_boosters = start_mass - booster_fuel - booster_dry
    if t <= t1:
        return mass_after_boosters - omega_stage1 * t
    mass_after_stage1 = mass_after_boosters - stage1_fuel - stage1_dry
    if t <= t2 + t1:
        dt = t - t1
        return mass_after_stage1 - omega_stage2 * dt
    mass_after_stage2 = mass_after_stage1 - stage2_fuel - stage2_dry
    if t2 + t1 < t <= t1 + t2 + t3 :
        dt = t - t2 - t1
        return mass_after_stage2 - omega_stage3 * dt
    return mass_after_stage2 - stage3_dry

def omega(t):
    if 0 <= t <= t0:
        return omega_booster + omega_stage1
    elif t0 < t <= t1:
        return omega_stage1
    elif t1 < t <= t1 + t2:
        return omega_stage2
    elif t1 + t2 < t <= t1 + t2 + t3:
        return omega_stage3
    else:
        return 0.0

def q(t):
    if 0 <= t <= t0:
        return 1667.1 + 2451
    elif t0 < t <= t1:
        return 3138.1
    elif t1 < t <= t2 + t1:
        return 3383.3
    elif t2 + t1 < t <= t2 + t1 + t3:
        return 3089.1
    else:
        return 0.0

def alpha(t):
    return (np.pi/2) * np.exp(-t/53)

def gravity(y):
    return G * M / (R + y)**2

def dynamics(X, t):
    x, y, vx, vy = X
    mass = m(t)
    if mass < 1.0:
        mass = 1.0
    a = alpha(t)
    g_local = gravity(y)
    w = omega(t)
    Q = q(t)

    # Тяга:
    F_thrust = w * Q
    F_thrust_x = F_thrust * np.cos(a)
    dvx_dt = (F_thrust_x) / mass

    if a > 0.06:
        F_thrust_y = F_thrust * np.sin(a)
        dvy_dt = (F_thrust_y) / mass - g_local
        dy_dt = vy
    else:
        dvy_dt = 0
        dy_dt = 0

    dx_dt = vx
    return [dx_dt, dy_dt, dvx_dt, dvy_dt]

if __name__ == '__main__':
    X0 = [0.0, 0.0, 0.0, 0.0]
    t_end = T
    t_points = np.linspace(0, t_end, 5000)

    sol = odeint(dynamics, X0, t_points)
    x_sol = sol[:, 0]
    y_sol = sol[:, 1]
    vx_sol = sol[:, 2]
    vy_sol = sol[:, 3]

    mass_sol = np.array([m(ti) for ti in t_points])
    tan_sol = np.array([alpha(ti) for ti in t_points])
    g_sol = np.array([gravity(ti) for ti in t_points])
    v_sol = np.sqrt(vx_sol**2 + vy_sol**2)
    omega_graf = np.array([omega(ti) for ti in t_points])

    fig, axs = plt.subplots(3, 3, figsize=(12, 10))
    axs[0][0].plot(t_points, v_sol)
    axs[0][0].set_xlabel("t, c")
    axs[0][0].set_ylabel("Скорость, м/с")
    axs[0][0].grid(True)
    axs[0][0].set_title("График скорости от времени")

    axs[0][1].plot(t_points, mass_sol)
    axs[0][1].set_xlabel("t, c")
    axs[0][1].set_ylabel("Масса, кг")
    axs[0][1].grid(True)
    axs[0][1].set_title("График массы от времени")

    axs[1][0].plot(t_points, y_sol)
    axs[1][0].set_xlabel("t, c")
    axs[1][0].set_ylabel("Высота, м")
    axs[1][0].grid(True)
    axs[1][0].set_title("График высоты от времени")

    axs[1][1].plot(x_sol, y_sol)
    axs[1][1].set_xlabel("x, м")
    axs[1][1].set_ylabel("y, м")
    axs[1][1].grid(True)
    axs[1][1].set_title("Перемещение в координатной плоскости")

    axs[2][1].plot(t_points, vx_sol)
    axs[2][1].set_xlabel("t, c")
    axs[2][1].set_ylabel("Vx, м/c")
    axs[2][1].grid(True)
    axs[2][1].set_title("Скорость по x")

    axs[2][0].plot(t_points, vy_sol)
    axs[2][0].set_xlabel("t, c")
    axs[2][0].set_ylabel("Vy, м/c")
    axs[2][0].grid(True)
    axs[2][0].set_title("Скорость по y")

    axs[1][2].plot(t_points, tan_sol)
    axs[1][2].set_xlabel("t, c")
    axs[1][2].set_ylabel("Угол, рад")
    axs[1][2].grid(True)
    axs[1][2].set_title("График угла от времени")

    axs[0][2].plot(t_points, g_sol)
    axs[0][2].set_xlabel("t, c")
    axs[0][2].set_ylabel("Ускорение, м/с^2")
    axs[0][2].grid(True)
    axs[0][2].set_title("График ускорения свободного падения")

    axs[2][2].plot(t_points, omega_graf)
    axs[2][2].set_xlabel("t, c")
    axs[2][2].set_ylabel("Омега, м/с")
    axs[2][2].grid(True)
    axs[2][2].set_title("График омеги от времени")


    plt.tight_layout()
    plt.show()
