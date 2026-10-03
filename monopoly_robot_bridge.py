"""
monopoly_robot_bridge.py
========================
Servidor de puente entre MATLAB (BOT-OPOLY Game Engine) y Python (UR5e Robot Control).

Este script escucha en un Socket TCP (localhost:5000) las jugadas transmitidas por MATLAB.
Recibe los datos del turno (movimientos de ficha, tiradas de dados, compras),
simula la recepción y ejecución del movimiento en el brazo robótico UR5e, y envía
un handshake de confirmación ("DONE") a MATLAB para que pueda continuar el juego.
"""

import os
import socket
import json
import time
from datetime import datetime

# Dentro de Docker se usa BRIDGE_HOST=0.0.0.0 para aceptar conexiones desde el host
HOST = os.environ.get("BRIDGE_HOST", "127.0.0.1")
PORT = int(os.environ.get("BRIDGE_PORT", "5000"))

TILE_NAMES = [
    "01: Salida (GO)", "02: Mediter. Ave.", "03: Caja de Comunidad", "04: Baltic Ave.",
    "05: Impuesto sobre Ingresos", "06: Reading Railroad", "07: Oriental Ave.", "08: Suerte (Chance)",
    "09: Vermont Ave.", "10: Connecticut Ave.", "11: Carcel / Visita", "12: St. Charles Place",
    "13: Compañia de Electricidad", "14: States Ave.", "15: Virginia Ave.", "16: Pennsylvania Railroad",
    "17: St. James Place", "18: Caja de Comunidad", "19: Tennessee Ave.", "20: New York Ave.",
    "21: Parada Libre", "22: Kentucky Ave.", "23: Suerte (Chance)", "24: Indiana Ave.",
    "25: Illinois Ave.", "26: B. & O. Railroad", "27: Atlantic Ave.", "28: Ventnor Ave.",
    "29: Compañia de Agua", "30: Marvin Gardens", "31: Ir a la Carcel", "32: Pacific Ave.",
    "33: North Carolina Ave.", "34: Caja de Comunidad", "35: Pennsylvania Ave.", "36: Short Line Railroad",
    "37: Suerte (Chance)", "38: Park Place", "39: Impuesto de Lujo", "40: Boardwalk"
]

def get_tile_name(tile_index):
    try:
        idx = int(tile_index) - 1
        if 0 <= idx < len(TILE_NAMES):
            return TILE_NAMES[idx]
    except Exception:
        pass
    return f"Casilla {tile_index}"

def start_bridge_server():
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((HOST, PORT))
    server_socket.listen(5)
    
    print("=" * 65)
    print("      PYTHON - SERVIDOR PUENTE MONOPOLY <-> UR5e ROBOT       ")
    print("=" * 65)
    print(f"Escuchando conexiones de MATLAB en {HOST}:{PORT}...")
    print("Presiona Ctrl+C para detener el servidor.\n")
    
    try:
        while True:
            client_socket, client_address = server_socket.accept()
            print(f"[{datetime.now().strftime('%H:%M:%S')}] Conexión aceptada desde MATLAB ({client_address[0]}:{client_address[1]})\n")
            
            client_file = client_socket.makefile('r', encoding='utf-8')
            
            while True:
                line = client_file.readline()
                if not line:
                    print(f"[{datetime.now().strftime('%H:%M:%S')}] MATLAB cerró la conexión.\n")
                    break
                
                line_str = line.strip()
                if not line_str:
                    continue
                
                try:
                    payload = json.loads(line_str)
                    process_game_event(payload)
                    
                    # Respuesta de confirmación terminada en \n a MATLAB
                    response = {
                        "status": "DONE",
                        "timestamp": datetime.now().isoformat(),
                        "message": "Acción procesada y movimiento robótico finalizado."
                    }
                    resp_json = json.dumps(response) + "\n"
                    client_socket.sendall(resp_json.encode('utf-8'))
                    
                except json.JSONDecodeError as e:
                    print(f"[ERROR] Error decodificando JSON de MATLAB: {e}")
                    error_resp = json.dumps({"status": "ERROR", "message": str(e)}) + "\n"
                    client_socket.sendall(error_resp.encode('utf-8'))
                    
            client_socket.close()
            
    except KeyboardInterrupt:
        print("\nServidor detenido manualmente por el usuario.")
    finally:
        server_socket.close()

def process_game_event(payload):
    turn = payload.get("turn", 0)
    player = payload.get("player", 0)
    from_tile = payload.get("from_tile", 1)
    to_tile = payload.get("to_tile", 1)
    cash = payload.get("cash", 0)
    net_worth = payload.get("net_worth", 0)
    is_jailed = payload.get("is_jailed", False)
    
    from_name = get_tile_name(from_tile)
    to_name = get_tile_name(to_tile)
    
    print("-" * 65)
    print(f" JUGADA RECIBIDA | Turno: {turn} | Jugador P{player}")
    print("-" * 65)
    print(f" -> Posición Anterior : {from_name}")
    print(f" -> Posición Nueva    : {to_name}")
    print(f" -> Efectivo Actual   : ${cash}")
    print(f" -> Valor Neto        : ${net_worth}")
    if is_jailed:
        print(" -> Estado            : EN LA CÁRCEL 🔒")
        
    print(f"\n [UR5e ROBOT CONTROL] Simulando movimiento del brazo:")
    print(f"   1. Calculando trayectoria IK de [{from_name}] a [{to_name}]...")
    print(f"   2. Gripper: Descendiendo y sujetando ficha del Jugador P{player}...")
    print(f"   3. Moviendo ficha por encima del tablero...")
    print(f"   4. Gripper: Colocando ficha en {to_name} y liberando...")
    print(f"   5. Brazo UR5e retornando a posición HOME de reposo.")
    
    time.sleep(0.3)
    print(" -> Movimiento completado con éxito! Enviando 'DONE' a MATLAB...\n")

if __name__ == "__main__":
    start_bridge_server()
