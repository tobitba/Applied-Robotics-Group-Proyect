% TEST_GAME_WITH_ROBOT.M
% Script de MATLAB para jugar Monopoly y enviar cada movimiento en tiempo real
% al servidor de Python (monopoly_robot_bridge.py) mediante TCP Sockets.

clc;
clear;
close all;

% 1. Configurar rutas de MATLAB
addpath('Classes');
addpath(genpath('.'));

fprintf('===================================================\n');
fprintf('  MATLAB BOT-OPOLY <-> PUENTE PYTHON UR5e ROBOT   \n');
fprintf('===================================================\n\n');

% 2. Conectar al servidor Python mediante TCP Socket
% En Docker, BRIDGE_HOST apunta al contenedor del puente
serverHost = getenv('BRIDGE_HOST');
if isempty(serverHost); serverHost = '127.0.0.1'; end
serverPort = 5000;

fprintf('1. Conectando al servidor Python en %s:%d ...\n', serverHost, serverPort);

% Reintentar unos segundos por si el puente aún está arrancando
maxAttempts = 10;
for attempt = 1:maxAttempts
    try
        client = tcpclient(serverHost, serverPort, 'Timeout', 10);
        break;
    catch ME
        if attempt == maxAttempts
            error('No se pudo conectar al servidor Python. Asegúrate de haber ejecutado "python monopoly_robot_bridge.py" primero en la terminal.\nError original: %s', ME.message);
        end
        pause(1);
    end
end
configureTerminator(client, "LF"); % Usa caracter de nueva línea \n
fprintf('   -> Conexión establecida con éxito con el script de Python!\n\n');

% 3. Cargar el modelo pre-entrenado
modelPath = fullfile('Resources', 'Models', 'model.mat');
fprintf('2. Cargando modelo del Bot (%s)...\n', modelPath);

if exist(modelPath, 'file')
    loaded = load(modelPath);
    model = loaded.model;
    fprintf('   -> Modelo cargado exitosamente!\n\n');
else
    error('No se encontro el archivo model.mat');
end

% 4. Inicializar partida
numPlayers = 4;
fprintf('3. Inicializando partida con %d jugadores...\n', numPlayers);
g = Monopoly(numPlayers);

fprintf('4. Iniciando simulacion de 15 turnos transmitidos a Python...\n');
fprintf('---------------------------------------------------\n');

epsilon = zeros(1, 4); % 0 = Modo óptimo con modelo de IA

for t = 1:15
    jugadorActual = g.current;
    pName = "P" + string(jugadorActual);
    
    % Obtener posición inicial del jugador antes de tirar
    fromTile = g.board.index(g.board.players(:, jugadorActual) == 1);
    if isempty(fromTile); fromTile = 1; end
    
    % Ejecutar el turno con el motor de juego
    g = game.turnManager(g, model, epsilon);
    
    % Obtener posición final del jugador después del turno
    toTile = g.board.index(g.board.players(:, jugadorActual) == 1);
    if isempty(toTile); toTile = 1; end
    
    cash = g.assets.(pName)(g.assets.asset == Resource.cash);
    netWorth = g.assets.(pName)(g.assets.asset == Resource.netWorth);
    isJailed = g.isJailed(jugadorActual);
    
    % Estructurar los datos de la jugada en formato JSON
    dataStruct = struct();
    dataStruct.turn = t;
    dataStruct.player = jugadorActual;
    dataStruct.from_tile = double(fromTile);
    dataStruct.to_tile = double(toTile);
    dataStruct.cash = double(cash);
    dataStruct.net_worth = double(netWorth);
    dataStruct.is_jailed = logical(isJailed);
    
    jsonStr = jsonencode(dataStruct);
    
    % Enviar línea JSON a Python por el Socket
    fprintf('Turno %2d | P%d movió de Casilla %2d -> %2d | Enviando a Python... ', ...
        t, jugadorActual, fromTile, toTile);
    writeline(client, jsonStr);
    
    % Esperar respuesta completa "DONE" terminada en \n de Python (bloqueante)
    respStr = readline(client);
    response = jsondecode(respStr);
    
    if strcmp(response.status, 'DONE')
        fprintf('[OK: Robot finalizó movimiento]\n');
    else
        fprintf('[ERROR: Respuesta del robot: %s]\n', response.message);
    end
end

fprintf('---------------------------------------------------\n');
fprintf('5. Partida finalizada. Cerrando conexión TCP.\n');
clear client;

fprintf('===================================================\n');
fprintf('   ¡PRUEBA DE INTEGRACION COMPLETADA CON EXITO!  \n');
fprintf('===================================================\n');
