try: 
    import sim
except:
    print("Erro ao importar biblioteca")

import time
import numpy as np

sim.simxFinish(-1)  # Fecha conexões anteriores

clientID = sim.simxStart('127.0.0.1', 19000, True, True, 5000, 5)

if clientID != -1:
    print("Conectado ao servidor!")

    # Pega os motores e o robô
    _, left_motor = sim.simxGetObjectHandle(clientID, 'dr12_leftJoint_', sim.simx_opmode_blocking)
    _, right_motor = sim.simxGetObjectHandle(clientID, 'dr12_rightJoint_', sim.simx_opmode_blocking)
    _, robo_handle = sim.simxGetObjectHandle(clientID, 'dr12', sim.simx_opmode_blocking)

    # Inicia leitura de posição e sensor
    sim.simxGetObjectPosition(clientID, robo_handle, -1, sim.simx_opmode_streaming)
    sim.simxGetObjectOrientation(clientID, robo_handle, -1, sim.simx_opmode_streaming)
    sim.simxGetStringSignal(clientID, 'Hokuyo', sim.simx_opmode_streaming)

    time.sleep(0.1)

    vaga_detectada = False
    inicio_vaga = None
    tempo_minimo_para_vaga = 0.8

    while True:
        err, data = sim.simxGetStringSignal(clientID, 'Hokuyo', sim.simx_opmode_buffer)
        _, pos = sim.simxGetObjectPosition(clientID, robo_handle, -1, sim.simx_opmode_buffer)
        _, orient = sim.simxGetObjectOrientation(clientID, robo_handle, -1, sim.simx_opmode_buffer)

        if err == sim.simx_return_ok and data:
            laser_data = sim.simxUnpackFloats(data)
            start_angle = -120
            angle_step = 240 / len(laser_data)

            direita = []
            frente = []
            for i, dist in enumerate(laser_data):
                ang = start_angle + i * angle_step
                if -20 <= ang <= 20:
                    frente.append(dist)
                elif -90 <= ang <= -45:
                    direita.append(dist)

            frente_livre = all(d > 0.5 for d in frente if d > 0.1)
            media_dir = np.mean([d for d in direita if d > 0.01]) if direita else 0

            # DETECÇÃO DE VAGA
            if not vaga_detectada:
                if media_dir > 1.10:
                    print(f"Média Direita: {media_dir:.2f}")

                    if inicio_vaga is None:
                        inicio_vaga = time.time()
                    else:
                        tempo_vaga = time.time() - inicio_vaga
                        if tempo_vaga >= tempo_minimo_para_vaga:
                            vaga_detectada = True
                            print("Vaga detectada!")
                            continue
                else:
                    inicio_vaga = None

            # ENTRA NA VAGA DIRETA PARA FRENTE
            if vaga_detectada:
                # Gira para a direita
                sim.simxSetJointTargetVelocity(clientID, left_motor, 1.0, sim.simx_opmode_streaming)
                sim.simxSetJointTargetVelocity(clientID, right_motor, -1.0, sim.simx_opmode_streaming)
                time.sleep(2.0)

                # Anda para frente (entrando na vaga)
                sim.simxSetJointTargetVelocity(clientID, left_motor, 2.0, sim.simx_opmode_streaming)
                sim.simxSetJointTargetVelocity(clientID, right_motor, 2.0, sim.simx_opmode_streaming)
                time.sleep(1.5)

                # Para o robô
                sim.simxSetJointTargetVelocity(clientID, left_motor, 0, sim.simx_opmode_streaming)
                sim.simxSetJointTargetVelocity(clientID, right_motor, 0, sim.simx_opmode_streaming)
                break

            # Movimento normal se ainda não achou vaga
            if frente_livre:
                sim.simxSetJointTargetVelocity(clientID, left_motor, 2.0, sim.simx_opmode_streaming)
                sim.simxSetJointTargetVelocity(clientID, right_motor, 2.0, sim.simx_opmode_streaming)
            else:
                # Desvia de obstáculos
                sim.simxSetJointTargetVelocity(clientID, left_motor, 1.0, sim.simx_opmode_streaming)
                sim.simxSetJointTargetVelocity(clientID, right_motor, -1.0, sim.simx_opmode_streaming)

        time.sleep(0.05)

    # Finaliza motores e encerra conexão
    sim.simxSetJointTargetVelocity(clientID, left_motor, 0, sim.simx_opmode_streaming)
    sim.simxSetJointTargetVelocity(clientID, right_motor, 0, sim.simx_opmode_streaming)
    sim.simxFinish(clientID)

else:
    print("Não conectou ao CoppeliaSim.")
