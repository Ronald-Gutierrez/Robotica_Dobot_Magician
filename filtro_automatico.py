import cv2
import numpy as np
import time
import asyncio
import DobotDllType as dType

# Diccionario de estados de conexión
CON_STR = {
    dType.DobotConnect.DobotConnect_NoError: "DobotConnect_NoError",
    dType.DobotConnect.DobotConnect_NotFound: "DobotConnect_NotFound",
    dType.DobotConnect.DobotConnect_Occupied: "DobotConnect_Occupied"
}

# Cargar la biblioteca DLL y obtener el objeto CDLL correspondiente
api = dType.load()

# Conectar con el brazo Dobot en el puerto COM6 con velocidad de 115200 baudios
state = dType.ConnectDobot(api, "COM6", 115200)[0]
print("Connect status:", CON_STR[state])

# Esta variable controla si el procesamiento está en curso
procesando = False

# Función asíncrona para evaluar los bordes durante 3 segundos
async def evaluar_bordes(cap, pixel_threshold, tiempo_evaluacion=3):
    global procesando
    procesando = True  # Marcar que el procesamiento ha iniciado

    detecciones_true = 0
    detecciones_false = 0
    start_time = time.time()

    while time.time() - start_time < tiempo_evaluacion:
        ret, imagen = cap.read()

        if not ret:
            print("Error al capturar la imagen de la cámara")
            break

        # Convertir a escala de grises
        imagen_gris = cv2.cvtColor(imagen, cv2.COLOR_BGR2GRAY)

        # Aplicar Filtro Gaussiano para reducir el ruido
        imagen_suavizada = cv2.GaussianBlur(imagen_gris, (5, 5), 0)

        # Aplicar detector de bordes de Canny
        umbral_bajo = 40
        umbral_alto = 60
        bordes = cv2.Canny(imagen_suavizada, umbral_bajo, umbral_alto)



        # Contar píxeles detectados como bordes
        pixel_count = np.sum(bordes > 0)

        # Evaluar si el número de píxeles detectados como bordes supera el umbral
        if pixel_count > pixel_threshold:
            detecciones_true += 1
        else:
            detecciones_false += 1

        # Mostrar los bordes detectados en la ventana
        cv2.imshow('Bordes Detectados', bordes)

        # Pequeña pausa para no sobrecargar el procesamiento
        await asyncio.sleep(0.1)

    # Decidir el resultado basado en la mayoría
    if detecciones_true > detecciones_false:
        resultado_final = True
        print(f"Detección final: TRUE ({detecciones_true} frames positivos vs {detecciones_false} negativos)")
    else:
        resultado_final = False
        print(f"Detección final: FALSE ({detecciones_true} frames positivos vs {detecciones_false} negativos)")

    procesando = False  # Marcar que el procesamiento ha finalizado
    return resultado_final



def evalue(frame):
    bordes = cv2.Canny(frame, 50, 150)

    contornos, _ = cv2.findContours(bordes, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contornos:
        contorno_principal = max(contornos, key=cv2.contourArea)
        
        x, y, w, h = cv2.boundingRect(contorno_principal)
        print(x, y, w, h)

        margen = 10

        roi = frame[y+20 : y+h - 20, x+20 : x+w-20]
        
        # Aplica Canny al recorte
        canny_recorte = cv2.Canny(roi, 50, 150)
        cv2.imshow('Recorte', canny_recorte)

        pixel_count = np.sum(canny_recorte > 0)

        if(pixel_count > 300):
            print("Pixel count: ", pixel_count)
            return True
    return False




# Función principal que captura el evento de la barra espaciadora y llama a evaluar_bordes
async def main():
    global procesando
    cap = cv2.VideoCapture(1)

    if not cap.isOpened():
        print("Error al abrir la cámara")
        return

    while True:
        ret, frame = cap.read()

        if not ret:
            print("Error al capturar la imagen de la cámara")
            break

        # Mostrar el video en tiempo real
        cv2.imshow('Camara', frame)

        # Escuchar eventos del teclado
        key = cv2.waitKey(1) & 0xFF

        # Si el usuario presiona la barra espaciadora y no se está procesando, iniciar evaluación
        if key == ord(' ') and not procesando:
            print("Evaluando durante 3 segundos...")
            # Iniciar el procesamiento asíncrono de la evaluación
            #resultado = await evaluar_bordes(cap, pixel_threshold=800)
            resultado = evalue(frame)
            # Aquí puedes utilizar el resultado para interactuar con el robot
            if resultado:
                print("Ejecutar acción positiva para el robot....................................")

                # Verificar si la conexión fue exitosa
                if state == dType.DobotConnect.DobotConnect_NoError:
    
                    # Detener la ejecución de la cola de comandos (por si estaba ejecutando previamente)
                    dType.SetQueuedCmdStopExec(api)
                    
                    # Limpiar la cola de comandos antes de enviar instrucciones nuevas
                    dType.SetQueuedCmdClear(api)
                    
                    # Obtener la posición actual del brazo Dobot
                    current_pos = dType.GetPose(api)
                    print("current_pos", current_pos)
                    # Movimiento total en el eje X (2 cm = 20 mm)


                    dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, current_pos[0]+30, current_pos[1], current_pos[2], 0, isQueued=1)
                    dType.SetQueuedCmdStartExec(api)
                    dType.dSleep(1000)
                    dType.SetQueuedCmdStopExec(api)

                    current_pos[0] = current_pos[0] + 30

                    #Succion
                    dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, current_pos[0], current_pos[1], current_pos[2] - 60, 0, isQueued=1)
                    dType.SetQueuedCmdStartExec(api)
                    dType.dSleep(1000)
                    dType.SetQueuedCmdStopExec(api)

                    dType.SetEndEffectorSuctionCup(api, enableCtrl=1, on=1, isQueued=0)

                    for i in range(0,150, 10):
                        new_x = current_pos[0] - i
                        new_y = current_pos[1] - i
                        new_z = current_pos[2]

                        dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, new_x, new_y, new_z, 0, isQueued=1)
                        dType.SetQueuedCmdStartExec(api)
                        dType.dSleep(1000)
                        dType.SetQueuedCmdStopExec(api)

                    dType.SetEndEffectorSuctionCup(api, enableCtrl=1, on=0, isQueued=0)

                    dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, current_pos[0], current_pos[1], current_pos[2], 0, isQueued=1)
                    dType.SetQueuedCmdStartExec(api)
                    dType.dSleep(1000)
                    dType.SetQueuedCmdStopExec(api)

                    dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, current_pos[0] - 30, current_pos[1], current_pos[2], 0, isQueued=1)
                    dType.SetQueuedCmdStartExec(api)
                    dType.dSleep(1000)
                    dType.SetQueuedCmdStopExec(api)

                    current_pos[0] = current_pos[0] - 30

                    # for i in range(0,150, 10):
                    #     new_x = current_pos[0] + i
                    #     new_y = current_pos[1] + i
                    #     new_z = current_pos[2]

                    #     dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, new_x, new_y, new_z, 0, isQueued=1)
                    #     dType.SetQueuedCmdStartExec(api)
                    #     dType.dSleep(1000)
                    #     dType.SetQueuedCmdStopExec(api)


            else:
                print("Ejecutar acción negativa para el robot....................................")

                if state == dType.DobotConnect.DobotConnect_NoError:
    
                    # Detener la ejecución de la cola de comandos (por si estaba ejecutando previamente)
                    dType.SetQueuedCmdStopExec(api)
                    
                    # Limpiar la cola de comandos antes de enviar instrucciones nuevas
                    dType.SetQueuedCmdClear(api)
                    
                    # Obtener la posición actual del brazo Dobot
                    current_pos = dType.GetPose(api)
                    print("current_pos", current_pos)
                    # Movimiento total en el eje X (2 cm = 20 mm)

                    dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, current_pos[0]+30, current_pos[1], current_pos[2], 0, isQueued=1)
                    dType.SetQueuedCmdStartExec(api)
                    dType.dSleep(1000)
                    dType.SetQueuedCmdStopExec(api)

                    current_pos[0] = current_pos[0] + 30

                    #Succion
                    dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, current_pos[0], current_pos[1], current_pos[2] - 60, 0, isQueued=1)
                    dType.SetQueuedCmdStartExec(api)
                    dType.dSleep(1000)
                    dType.SetQueuedCmdStopExec(api)

                    dType.SetEndEffectorSuctionCup(api, enableCtrl=1, on=1, isQueued=0)

                    for i in range(0,150, 10):
                        new_x = current_pos[0] - i
                        new_y = current_pos[1] + i
                        new_z = current_pos[2]

                        dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, new_x, new_y, new_z, 0, isQueued=1)
                        dType.SetQueuedCmdStartExec(api)
                        dType.dSleep(1000)
                        dType.SetQueuedCmdStopExec(api)

                    dType.SetEndEffectorSuctionCup(api, enableCtrl=1, on=0, isQueued=0)

                    dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, current_pos[0], current_pos[1], current_pos[2], 0, isQueued=1)
                    dType.SetQueuedCmdStartExec(api)
                    dType.dSleep(1000)
                    dType.SetQueuedCmdStopExec(api)

                    dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, current_pos[0] - 30, current_pos[1], current_pos[2], 0, isQueued=1)
                    dType.SetQueuedCmdStartExec(api)
                    dType.dSleep(1000)
                    dType.SetQueuedCmdStopExec(api)

                    current_pos[0] = current_pos[0] - 30

                    # for i in range(0,150, 10):
                    #     new_x = current_pos[0] - i
                    #     new_y = current_pos[1] - i
                    #     new_z = current_pos[2]

                    #     dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, new_x, new_y, new_z, 0, isQueued=1)
                    #     dType.SetQueuedCmdStartExec(api)
                    #     dType.dSleep(1000)
                    #     dType.SetQueuedCmdStopExec(api)

        # Salir si se presiona la tecla 'q'
        if key == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

# Ejecutar el bucle principal
asyncio.run(main())
