"""
master_orchestrator.py
======================
Proceso Máster / Orquestador Central del Proyecto Robo-Poly.

Este script es el motor principal del sistema:
1. Controla el flujo de turnos y la consola interactiva de la partida.
2. Invoca al módulo de Visión por Computadora (CV) para leer los dados físicos.
3. Pregunta al usuario por terminal si la lectura del CV es correcta (s/n) o requiere corrección.
4. Envía el dado validado a MATLAB (matlab_rules_server.m) para aplicar las reglas de Monopoly y actualizar el Banco digital.
5. Simula el control robótico del UR5e para mover las fichas del Bot.
"""

import socket
import json
import time
import sys
from datetime import datetime
from ComputerVision.cv_service import capture_dice_reading

HOST = "127.0.0.1"
RULES_PORT = 5001  # Puerto donde se conecta el servidor de reglas de MATLAB

def start_master_orchestrator():
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((HOST, RULES_PORT))
    server_socket.listen(1)
    
    print("=" * 70)
    print("       ROBO-POLY: ORQUESTADOR CENTRAL MÁSTER (PYTHON)       ")
    print("=" * 70)
    print(f"1. Esperando conexión del Servidor de Reglas de MATLAB en {HOST}:{RULES_PORT}...")
    print("   -> Por favor, ejecuta 'matlab_rules_server' en la consola de MATLAB ahora.\n")
    
    matlab_socket, addr = server_socket.accept()
    print(f" -> Conexión establecida con MATLAB desde {addr[0]}:{addr[1]}!\n")
    
    matlab_file = matlab_socket.makefile('r', encoding='utf-8')
    
    def send_to_matlab(payload):
        json_line = json.dumps(payload) + "\n"
        matlab_socket.sendall(json_line.encode('utf-8'))
        resp_line = matlab_file.readline()
        if not resp_line:
            return {"status": "ERROR", "message": "MATLAB cerró la conexión"}
        return json.loads(resp_line.strip())

    print("=" * 70)
    print("   ¡SISTEMA LISTO! INICIANDO PARTIDA REAL CON VALIDACIÓN DE DADOS")
    print("=" * 70)
    
    current_player = 1
    num_players = 4
    turn_counter = 1
    
    try:
        while True:
            print(f"\n====================== TURNO {turn_counter} ======================")
            is_human = (current_player == 1)
            player_type = "HUMANO (Tú)" if is_human else f"BOT P{current_player} (UR5e Robot)"
            print(f" >>> Jugador Actual: P{current_player} - [{player_type}] <<<")
            
            # -------------------------------------------------------------
            # PASO 1: OBTENER DADOS DE VISIÓN POR COMPUTADORA (CV)
            # -------------------------------------------------------------
            print("\n[PASO 1] Iniciando escaneo de cámara cenital para detectar dados...")
            
            dice_list, cv_sum = capture_dice_reading(max_seconds=3.0, camera_index=0, show_preview=True)
            
            if not dice_list:
                print(" [CV NOTICE] No se detectaron dados en la cámara. Generando dado por defecto (ej: [3, 4]).")
                dice_list = [3, 4]
                cv_sum = 7
                
            # -------------------------------------------------------------
            # PASO 2: CONFIRMACIÓN / VALIDACIÓN HUMANA EN TERMINAL
            # -------------------------------------------------------------
            print("\n" + "-" * 70)
            print(" [VALIDACIÓN DE VISIÓN] RESULTADO DE LA CÁMARA DETECTADO:")
            print(f"  -> Dados leídos  : {dice_list}")
            print(f"  -> Suma calculada : {cv_sum}")
            print("-" * 70)
            
            confirmed_sum = cv_sum
            ans = input(" ¿Es correcto el resultado detectado por la cámara? (s/n) [Defecto: s]: ").strip().lower()
            
            if ans == 'n':
                while True:
                    try:
                        manual_val = int(input(" Ingrese la suma correcta de los dados físicos (2-12): "))
                        if 2 <= manual_val <= 12:
                            confirmed_sum = manual_val
                            print(f" -> Valor corregido manualmente a: {confirmed_sum}")
                            break
                        else:
                            print(" Por favor ingrese un valor entre 2 y 12.")
                    except ValueError:
                        print(" Entrada inválida. Ingrese un número entero.")
            else:
                print(f" -> Resultado confirmado: {confirmed_sum}")
                
            # -------------------------------------------------------------
            # PASO 3: ENVIAR DADO VALIDADO AL MOTOR DE REGLAS EN MATLAB
            # -------------------------------------------------------------
            print(f"\n[PASO 3] Enviando dado verificado ({confirmed_sum}) a MATLAB...")
            
            req = {
                "cmd": "PROCESS_MOVE",
                "player": current_player,
                "dice_roll": confirmed_sum
            }
            
            res = send_to_matlab(req)
            
            if res.get("status") != "OK":
                print(f"[ERROR MATLAB] {res.get('message')}")
                break
                
            from_tile = res.get("from_tile")
            to_tile = res.get("to_tile")
            prop_name = res.get("prop_name")
            tile_type = res.get("tile_type")
            cash = res.get("cash")
            net_worth = res.get("net_worth")
            can_buy = res.get("can_buy", False)
            price = res.get("purchase_price", 0)
            
            print(f" -> Avance en el tablero: Casilla {from_tile} -> Casilla {to_tile} ({prop_name})")
            print(f" -> Estado Financiero   : Efectivo = ${cash} | Valor Neto = ${net_worth}")
            
            # -------------------------------------------------------------
            # PASO 4: INTERACCIÓN O MOVIMIENTO ROBÓTICO
            # -------------------------------------------------------------
            if is_human:
                if can_buy:
                    buy_ans = input(f"\n ¿Deseas comprar {prop_name} por ${price}? (s/n): ").strip().lower()
                    if buy_ans == 's':
                        buy_req = {"cmd": "BUY_PROPERTY", "player": 1, "tile_index": to_tile}
                        buy_res = send_to_matlab(buy_req)
                        print(f" ¡Propiedad comprada! Nuevo efectivo: ${buy_res.get('cash')}")
                    else:
                        print(" Decidiste no comprar la propiedad.")
            else:
                print(f"\n [UR5e ROBOT CONTROL] Moviendo la ficha física del Bot P{current_player}:")
                print(f"   -> Ejecutando Pick & Place de Casilla {from_tile} a Casilla {to_tile}...")
                time.sleep(0.5)
                print("   -> Movimiento robótico finalizado.")
                
            current_player = res.get("next_player", (current_player % num_players) + 1)
            turn_counter += 1
            
            cont = input("\n Presiona ENTER para pasar al siguiente turno (o 'q' para salir): ").strip().lower()
            if cont == 'q':
                print("\nFinalizando partida por solicitud del usuario...")
                send_to_matlab({"cmd": "QUIT"})
                break
                
    except KeyboardInterrupt:
        print("\nOrquestador detenido manualmente por teclado.")
    finally:
        matlab_socket.close()
        server_socket.close()
        print("Orquestador Máster cerrado.")

if __name__ == "__main__":
    start_master_orchestrator()
