# импортируем библиотеки
import math
import time
import krpc
import matplotlib.pyplot as plt

# задаём значение гравитационного манёвра и орбиты
turn_start_altitude = 250
turn_end_altitude = 45000
target_altitude = 80000

# название миссии
conn = krpc.connect(name='Запуск на орбиту')

vessel = conn.space_center.active_vessel

# получаем потоки нужных значений высота, апогей, скорость, массу, поток для ступени с ускорителями
ut = conn.add_stream(getattr, conn.space_center, 'ut')
altitude = conn.add_stream(getattr, vessel.flight(), 'mean_altitude')
apoapsis = conn.add_stream(getattr, vessel.orbit, 'apoapsis_altitude')
speed = conn.add_stream(getattr, vessel.flight(vessel.orbit.body.reference_frame), 'speed')
mass = conn.add_stream(getattr, vessel, 'mass')

stage_5_resources = vessel.resources_in_decouple_stage(stage=5, cumulative=False)
srb_fuel = conn.add_stream(stage_5_resources.amount, 'SolidFuel')

# Данные для графиков
time_data = []
altitude_data = []
speed_data = []
mass_data = []

# обновление данных с заданным интервалом времени
last_record_time = 0
record_interval = 0.5  # seconds

def record_data():
    global last_record_time
    current_time = ut()
    if current_time - last_record_time >= record_interval:
        time_data.append(current_time)
        altitude_data.append(altitude())
        speed_data.append(speed())
        mass_data.append(mass())
        last_record_time = current_time

# Настройка до запуска
vessel.control.sas = False
vessel.control.throttle = 1.0

# Ожидание
print('3...')
time.sleep(1)
print('2...')
time.sleep(1)
print('1...')
time.sleep(1)
print('Launch!')

# Активация первой ступени
vessel.control.activate_next_stage()
vessel.control.activate_next_stage()
vessel.auto_pilot.engage()
vessel.auto_pilot.target_pitch_and_heading(90, 90)


# Основной цикл подъема
srbs_separated = False
turn_angle = 0
while True:
    record_data()

    # Гравитационный манёвр
    if altitude() > turn_start_altitude and altitude() < turn_end_altitude:
        frac = ((altitude() - turn_start_altitude) /
                (turn_end_altitude - turn_start_altitude))
        new_turn_angle = frac * 90
        if abs(new_turn_angle - turn_angle) > 0.5:
            turn_angle = new_turn_angle
            vessel.auto_pilot.target_pitch_and_heading(90-turn_angle, 90)

    # Отделение ускорителей
    if not srbs_separated:
        if srb_fuel() < 0.1:
            vessel.control.activate_next_stage()
            srbs_separated = True
            print('SRBs separated')

    # Уменьшайте скорость при приближении к апогею
    if apoapsis() > target_altitude*0.9:
        vessel.control.throttle = 0.0
        break

# Отключите двигатели при достижении нужного апогея
vessel.control.throttle = 0.25
while apoapsis() < target_altitude:
    record_data()
    pass

# Отделяем защитный купол
vessel.control.activate_next_stage()

print('Target apoapsis reached')
vessel.control.throttle = 0.0

# Ждём, пока не вышли из атмосферы
print('Coasting out of atmosphere')
while altitude() < 70500:
    record_data()
    pass


# Планируем манёвр по округлению орбиты
print('Planning circularization burn')
mu = vessel.orbit.body.gravitational_parameter
r = vessel.orbit.apoapsis
a1 = vessel.orbit.semi_major_axis
a2 = r
v1 = math.sqrt(mu*((2./r)-(1./a1)))
v2 = math.sqrt(mu*((2./r)-(1./a2)))
delta_v = v2 - v1
node = vessel.control.add_node(
    ut() + vessel.orbit.time_to_apoapsis, prograde=delta_v)

# Рассчитываем время работы двигателя
F = vessel.available_thrust
Isp = vessel.specific_impulse * 9.82
m0 = vessel.mass
m1 = m0 / math.exp(delta_v/Isp)
flow_rate = F / Isp
burn_time = (m0 - m1) / flow_rate


# Ориентируем корабль
print('Orientating ship for circularization burn')
vessel.auto_pilot.reference_frame = node.reference_frame
vessel.auto_pilot.target_direction = (0, 1, 0)
vessel.auto_pilot.wait()

# Ждём манёвр
print('Waiting until circularization burn')
burn_ut = ut() + vessel.orbit.time_to_apoapsis - (burn_time/2.)
lead_time = 5
conn.space_center.warp_to(burn_ut - lead_time)
# отделяем первую ступень (уже пустую) и запускаем двигатель
vessel.control.activate_next_stage()
vessel.control.activate_next_stage()

# Готовы выполнить манёвр
print('Ready to execute burn')
time_to_apoapsis = conn.add_stream(getattr, vessel.orbit, 'time_to_apoapsis')
while time_to_apoapsis() - (burn_time/2.) > 0:
    record_data()
    pass

# Выполняется манёвр и ждём
print('Executing burn')
vessel.control.throttle = 1.0
time.sleep(2 * burn_time - 0.1)

# Манёвр выполнен
print('Fine tuning')
vessel.control.throttle = 0.05
remaining_burn = conn.add_stream(node.remaining_burn_vector, node.reference_frame)

# Выпускаем спутник
vessel.control.activate_next_stage()
node.remove()

print('Launch complete')

# Отображаем записанные данные на графике
plt.figure(figsize=(12, 8))

# График высоты
plt.subplot(3, 1, 1)
plt.plot(time_data, altitude_data, label='Altitude (m)', color='blue')
plt.xlabel('Time (s)')
plt.ylabel('Altitude (m)')
plt.legend()

# График скорости
plt.subplot(3, 1, 2)
plt.plot(time_data, speed_data, label='Speed (m/s)', color='green')
plt.xlabel('Time (s)')
plt.ylabel('Speed (m/s)')
plt.legend()

# График массы
plt.subplot(3, 1, 3)
plt.plot(time_data, mass_data, label='Mass (kg)', color='red')
plt.xlabel('Time (s)')
plt.ylabel('Mass (kg)')
plt.legend()

# Показать все графики
plt.tight_layout()
plt.show()
