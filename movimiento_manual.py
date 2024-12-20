import threading
import keyboard  # Necesitarás instalar esta biblioteca con 'pip install keyboard'
import DobotDllType as dType
import math
import time

# Límite de movimiento en mm
X_MIN = -200
X_MAX = 200
Y_MIN = -200
Y_MAX = 200
Z_MIN = -75
Z_MAX = 400

# Movimiento en mm para teclas de dirección
delta_movement = 25  # 2 cm
delta_rotation = 10  # Ángulo de rotación en grados

# Estado del succionador
succionador_activado = False
keys_pressed = set()

# Variable para la posición inicial
posicion_inicial = None

# Funciones auxiliares para el control del brazo Dobot
def mover_brazo(api, current_pos, dx=0, dy=0, dz=0):
    new_x = max(X_MIN, min(X_MAX, current_pos[0] + dx))
    new_y = max(Y_MIN, min(Y_MAX, current_pos[1] + dy))
    new_z = max(Z_MIN, min(Z_MAX, current_pos[2] + dz))
    print("Current pos:", current_pos, "-> New pos:", new_x, new_y, new_z)

    dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, new_x, new_y, new_z, 0, isQueued=1)
    dType.SetQueuedCmdStartExec(api)
    dType.dSleep(500)
    dType.SetQueuedCmdStopExec(api)

    return new_x, new_y, new_z

def activar_succionador(api, activar=True):
    global succionador_activado
    if activar and not succionador_activado:
        dType.SetEndEffectorSuctionCup(api, enableCtrl=1, on=1, isQueued=0)
        succionador_activado = True
        print("Succionador activado.")
    elif not activar and succionador_activado:
        dType.SetEndEffectorSuctionCup(api, enableCtrl=1, on=0, isQueued=0)
        succionador_activado = False
        print("Succionador desactivado.")

def volver_a_posicion_inicial(api, inicial_pos):
    # Solo tomamos las coordenadas x, y, z
    inicial_x, inicial_y, inicial_z = inicial_pos[:3]
    dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, inicial_x, inicial_y, inicial_z, 0, isQueued=1)
    dType.SetQueuedCmdStartExec(api)
    dType.dSleep(500)
    dType.SetQueuedCmdStopExec(api)
    print(f"Volviendo a la posición inicial: {inicial_x}, {inicial_y}, {inicial_z}")

def escanear_en_curva(api, inicial_pos):
    """
    Movimiento en curva más amplio hacia la izquierda y luego hacia la derecha, terminando en la posición inicial.
    """
    # Coordenadas iniciales
    inicial_x, inicial_y, inicial_z = inicial_pos[:3]
    radio = 100  # Radio de la curva (ampliado)
    pasos = 20   # Más pasos para mayor suavidad en la curva

    # Movimiento hacia la izquierda (arco de 180 grados)
    for angulo in range(0, 181, int(180 / pasos)):  # De 0° a 180° en pasos
        radianes = math.radians(angulo)
        new_x = inicial_x - radio * math.sin(radianes)
        new_y = inicial_y + radio * (1 - math.cos(radianes))
        dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, new_x, new_y, inicial_z, 0, isQueued=1)
        dType.SetQueuedCmdStartExec(api)
        dType.dSleep(200)

    # Movimiento hacia la derecha (arco de 180 grados)
    for angulo in range(180, -1, -int(180 / pasos)):  # De 180° a 0° en pasos
        radianes = math.radians(angulo)
        new_x = inicial_x + radio * math.sin(radianes)
        new_y = inicial_y + radio * (1 - math.cos(radianes))
        dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, new_x, new_y, inicial_z, 0, isQueued=1)
        dType.SetQueuedCmdStartExec(api)
        dType.dSleep(200)

    # Volver a la posición inicial
    volver_a_posicion_inicial(api, inicial_pos)

# Función para manejar las teclas en un hilo independiente
def control_teclado(api, current_pos):
    global keys_pressed
    while True:
        try:
            if keyboard.is_pressed('w') and 'w' not in keys_pressed:
                current_pos[0], current_pos[1], current_pos[2] = mover_brazo(api, current_pos, dx=delta_movement)
                keys_pressed.add('w')
            elif keyboard.is_pressed('s') and 's' not in keys_pressed:
                current_pos[0], current_pos[1], current_pos[2] = mover_brazo(api, current_pos, dx=-delta_movement)
                keys_pressed.add('s')
            elif keyboard.is_pressed('a') and 'a' not in keys_pressed:
                current_pos[0], current_pos[1], current_pos[2] = mover_brazo(api, current_pos, dy=delta_movement)
                keys_pressed.add('a')
            elif keyboard.is_pressed('d') and 'd' not in keys_pressed:
                current_pos[0], current_pos[1], current_pos[2] = mover_brazo(api, current_pos, dy=-delta_movement)
                keys_pressed.add('d')
            elif keyboard.is_pressed('x') and 'x' not in keys_pressed:
                current_pos[0], current_pos[1], current_pos[2] = mover_brazo(api, current_pos, dz=-delta_movement)
                keys_pressed.add('x')
            elif keyboard.is_pressed('z') and 'z' not in keys_pressed:
                current_pos[0], current_pos[1], current_pos[2] = mover_brazo(api, current_pos, dz=delta_movement)
                keys_pressed.add('z')
            elif keyboard.is_pressed(' ') and ' ' not in keys_pressed:
                activar_succionador(api, not succionador_activado)
                keys_pressed.add(' ')
            elif keyboard.is_pressed('enter') and 'enter' not in keys_pressed:
                volver_a_posicion_inicial(api, posicion_inicial)
                keys_pressed.add('enter')
            elif keyboard.is_pressed('b') and 'b' not in keys_pressed:
                escanear_en_curva(api, posicion_inicial)
                keys_pressed.add('b')

            # Eliminar teclas de la lista al soltarlas
            for key in list(keys_pressed):
                if not keyboard.is_pressed(key):
                    keys_pressed.discard(key)

        except KeyboardInterrupt:
            print("Deteniendo control de teclado.")
            break

# Conexión al brazo Dobot
api = dType.load()
state = dType.ConnectDobot(api, "COM6", 115200)[0]
if state == dType.DobotConnect.DobotConnect_NoError:
    dType.SetQueuedCmdStopExec(api)
    dType.SetQueuedCmdClear(api)

    current_pos = list(dType.GetPose(api))
    posicion_inicial = current_pos.copy()

    print("Control Dobot iniciado. Usa WASD para mover, ENTER para regresar al inicio, B para curva.")

    # Iniciar el hilo para el control del teclado
    thread_teclado = threading.Thread(target=control_teclado, args=(api, current_pos))
    thread_teclado.daemon = True
    thread_teclado.start()

    # Esperar el final del programa
    try:
        while thread_teclado.is_alive():
            time.sleep(1)
    except KeyboardInterrupt:
        print("Finalizando control del Dobot.")

    dType.DisconnectDobot(api)
else:
    print("No se pudo conectar con el Dobot.")
