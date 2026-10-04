% MATLAB_RULES_SERVER.M
% Servidor de Reglas del Juego Monopoly para el Orquestador Central en Python.
% Escucha las peticiones del Orquestador Python, procesa las jugadas y devuelve el estado del juego.

clc;
clear;
close all;

% 1. Configurar rutas de MATLAB
addpath('Classes');
addpath(genpath('.'));

fprintf('===================================================\n');
fprintf('   MATLAB ENGINE - SERVIDOR DE REGLAS MONOPOLY    \n');
fprintf('===================================================\n\n');

% 2. Cargar el modelo pre-entrenado
modelPath = fullfile('Resources', 'Models', 'model.mat');
fprintf('1. Cargando modelo del Bot desde %s ...\n', modelPath);

if exist(modelPath, 'file')
    loaded = load(modelPath);
    model = loaded.model;
    fprintf('   -> Modelo cargado exitosamente!\n\n');
else
    error('No se encontro el archivo model.mat');
end

% 3. Inicializar el estado del Monopoly
numPlayers = 4;
g = Monopoly(numPlayers);
epsilon = zeros(1, numPlayers);
fprintf('2. Partida de Monopoly inicializada para %d jugadores.\n', numPlayers);

% 4. Conectar con el Orquestador Central en Python (Puerto 5001)
serverHost = '127.0.0.1';
serverPort = 5001;

fprintf('3. Conectando con el Orquestador Central (Python) en %s:%d ...\n', serverHost, serverPort);

try
    client = tcpclient(serverHost, serverPort, 'Timeout', 3600); % 1 hora de timeout para permitir interacción humana
    configureTerminator(client, "LF");
    warning('off', 'transportlib:client:readlineTimeout');
    warning('off', 'instrument:tcpclient:readlineTimeout');
    fprintf('   -> Conexión establecida con el Orquestador Central!\n\n');
catch ME
    error('No se pudo conectar con el Orquestador Python en el puerto %d. Ejecuta master_orchestrator.py primero.\nError: %s', serverPort, ME.message);
end

fprintf('4. Listo para recibir solicitudes de jugadas...\n');
fprintf('---------------------------------------------------\n');

while true
    try
        % Leer orden en formato JSON desde Python
        rawLine = readline(client);
        if isempty(rawLine)
            continue;
        end
        
        req = jsondecode(char(rawLine));
        
        if strcmp(req.cmd, 'PROCESS_MOVE')
            player = req.player;
            roll = req.dice_roll;
            pName = "P" + string(player);
            
            % Obtener casilla actual antes del movimiento
            fromTile = g.board.index(g.board.players(:, player) == 1);
            if isempty(fromTile); fromTile = 1; end
            
            if player == 1
                % Turno de Humano: mover ficha con el dado validado por CV
                [g, toTile] = g.moveToken(roll, true, true);
                g.current = mod(g.current, numPlayers) + 1;
            else
                % Turno de Bot: ejecutar turnManager con el modelo de IA (turnManager avanza g.current automáticamente)
                g = game.turnManager(g, model, epsilon);
                toTile = g.board.index(g.board.players(:, player) == 1);
                if isempty(toTile); toTile = 1; end
            end
            
            propName = string(g.board.property(toTile));
            tileType = string(g.board.tile(toTile));
            cash = g.assets.(pName)(g.assets.asset == Resource.cash);
            netWorth = g.assets.(pName)(g.assets.asset == Resource.netWorth);
            isOwned = g.board.isOwned(toTile);
            
            % Determinar si la propiedad se puede comprar
            canBuy = ~isOwned && (propName ~= "null");
            purchasePrice = 0;
            if canBuy
                purchasePrice = g.board.property(toTile).purchasePrice;
            end
            
            % Construir respuesta JSON para el Orquestador
            res = struct();
            res.status = 'OK';
            res.player = player;
            res.from_tile = double(fromTile);
            res.to_tile = double(toTile);
            res.prop_name = char(propName);
            res.tile_type = char(tileType);
            res.cash = double(cash);
            res.net_worth = double(netWorth);
            res.is_owned = logical(isOwned);
            res.can_buy = logical(canBuy);
            res.purchase_price = double(purchasePrice);
            res.is_jailed = logical(g.isJailed(player));
            res.next_player = g.current;
            
            % Enviar respuesta a Python
            writeline(client, jsonencode(res));
            fprintf('Jugada procesada: P%d movió %d -> %d (%s) | Efectivo: $%d\n', ...
                player, fromTile, toTile, propName, cash);
                
        elseif strcmp(req.cmd, 'BUY_PROPERTY')
            player = req.player;
            tileIdx = req.tile_index;
            g = g.buyProperty(g.board.property(tileIdx), player);
            pName = "P" + string(player);
            cash = g.assets.(pName)(g.assets.asset == Resource.cash);
            
            res = struct('status', 'OK', 'message', 'Propiedad comprada exitosamente', 'cash', double(cash));
            writeline(client, jsonencode(res));
            fprintf('P%d compró la propiedad en casilla %d.\n', player, tileIdx);
            
        elseif strcmp(req.cmd, 'QUIT')
            fprintf('Recibida orden de cierre. Saliendo...\n');
            break;
        end
        
    catch ME
        fprintf('[ERROR] Exception en el servidor de reglas: %s\n', ME.message);
        errRes = struct('status', 'ERROR', 'message', ME.message);
        writeline(client, jsonencode(errRes));
    end
end

clear client;
fprintf('Servidor de Reglas de MATLAB finalizado.\n');
