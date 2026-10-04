"""
cv_service.py
=============
Módulo de servicio de Visión por Computadora para el proyecto Robo-Poly.
Encapsula la detección de dados mediante OpenCV y DBSCAN, capturando
fotogramas de la cámara cenital y devolviendo la lista de dados y su suma.
"""

import cv2
import numpy as np
import time
from ComputerVision.diceDetection import get_blobs, get_dice_from_blobs, overlay_info

def capture_dice_reading(max_seconds=4.0, camera_index=0, show_preview=True):
    """
    Abre la cámara cenital, captura fotogramas durante max_seconds para permitir
    que los dados se detengan, y retorna la lista de valores de dados detectados y su suma.
    
    Retorna:
      dice_values: list[int] (ej: [3, 4])
      total_sum: int (ej: 7)
    """
    cap = cv2.VideoCapture(camera_index, cv2.CAP_DSHOW)
    if not cap.isOpened():
        print("[CV SERVICE WARNING] No se pudo abrir la cámara index", camera_index)
        return [], 0

    start_time = time.time()
    last_detected_dice = []
    
    print(f"[CV SERVICE] Leyendo cámara por {max_seconds} segundos para estabilizar dados...")
    
    while (time.time() - start_time) < max_seconds:
        ret, frame = cap.read()
        if not ret or frame is None:
            continue
            
        blobs = get_blobs(frame)
        dice_info = get_dice_from_blobs(blobs)
        
        if dice_info:
            # Extraer solo los valores enteros de cada dado (cantidad de pips)
            last_detected_dice = [int(d[0]) for d in dice_info]
            
        if show_preview:
            overlay_info(frame, dice_info, blobs)
            cv2.putText(frame, "Analizando dados...", (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
            cv2.imshow("CV Dice Detector - Robo-Poly", frame)
            cv2.waitKey(30)
            
    cap.release()
    if show_preview:
        cv2.destroyAllWindows()
        
    total_sum = sum(last_detected_dice)
    return last_detected_dice, total_sum
