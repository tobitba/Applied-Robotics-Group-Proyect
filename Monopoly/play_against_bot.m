% PLAY_AGAINST_BOT.M
% Partida Interactiva: Humano (Jugador 1) vs. Bot/Robot UR5e (Jugador 2)

clc;
clear;
close all;

% 1. Configurar rutas de MATLAB
addpath('Classes');
addpath(genpath('.'));

fprintf('===================================================\n');
fprintf('    MONOPOLY: HUMANO vs. BOT (ROBOT UR5e)         \n');
fprintf('===================================================\n\n');

% 2. Conectar al servidor Python (opcional, si está corriendo)
serverHost = '127.0.0.1';
serverPort = 5000;
robotConnected = false;

try
    client = tcpclient(serverHost, serverPort, 'Timeout', 5);
    configureTerminator(client, "LF");
    robotConnected = true;
    fprintf(' -> Conectado al servidor Python (Robot UR5e activo!)\n\n');
catch
    fprintf(' -> Nota: Servidor Python no detectado. Se jugara en modo solo consola.\n\n');
end

% 3. Cargar el modelo pre-entrenado del Bot
modelPath = fullfile('Resources', 'Models', 'model.mat');
if exist(modelPath, 'file')
    loaded = load(modelPath);
    model = loaded.model;
    fprintf(' -> Modelo de IA del Bot cargado exitosamente!\n\n');
else
    error('No se encontro el archivo model.mat');
end

% 4. Crear partida para 2 jugadores (P1 = Humano, P2 = Bot)
g = Monopoly(2);
epsilon = zeros(1, 2);

fprintf('¡Partida iniciada! Jugador 1: HUMANO | Jugador 2: BOT (Robot UR5e)\n');
fprintf('===================================================\n\n');

maxTurnos = 30;

for t = 1:maxTurnos
    fprintf('\n------------------ TURNO %d ------------------\n', t);
    
    if g.current == 1
        % =========================================================
        % TURNO DEL JUGADOR 1 (HUMANO)
        % =========================================================
        fprintf('>>> TURNO DE %s (HUMANO) <<<\n', 'P1');
        
        fromTile = g.board.index(g.board.players(:, 1) == 1);
        cashP1 = g.assets.P1(g.assets.asset == Resource.cash);
        netP1 = g.assets.P1(g.assets.asset == Resource.netWorth);
        fprintf('Tus activos actuales: Efectivo = $%d | Valor Neto = $%d\n', cashP1, netP1);
        
        input('Presiona ENTER para lanzar los dados...', 's');
        
        % Lanzar dados
        [roll, isDouble] = g.roll();
        fprintf('Has sacado un [%d] en los dados!\n', roll);
        
        % Mover ficha
        [g, toTile] = g.moveToken(roll, true, true);
        propName = string(g.board.property(toTile));
        tileType = string(g.board.tile(toTile));
        
        fprintf('Te has movido a la Casilla %d (%s / %s)\n', toTile, propName, tileType);
        
        % Si la propiedad no tiene dueño y es comprable
        if ~g.board.isOwned(toTile) && propName ~= "null"
            precio = g.board.property(toTile).purchasePrice;
            if cashP1 >= precio
                opcion = input(sprintf('¿Deseas comprar %s por $%d? (1: Si, 0: No): ', propName, precio));
                if opcion == 1
                    g = g.buyProperty(g.board.property(toTile), 1);
                    fprintf('¡Has comprado %s!\n', propName);
                else
                    fprintf('Decidiste no comprar la propiedad.\n');
                end
            else
                fprintf('No tienes suficiente efectivo ($%d requeridos) para comprar %s.\n', precio, propName);
            end
        elseif g.board.isOwned(toTile) && g.board.owner(toTile) == 2
            % Pagar renta al Bot
            dueno = g.board.owner(toTile);
            renta = g.board.property(toTile).rent(g.board.numHouses(toTile) + 1);
            fprintf('¡Caiste en la propiedad del Bot! Pagas $%d de renta.\n', renta);
            [g, ~] = g.payCash(renta, 1, Transaction.cashPlayerToPlayer, 2);
        end
        
        % Pasar el turno al Bot
        g.current = 2;
        
    else
        % =========================================================
        % TURNO DEL JUGADOR 2 (BOT / ROBOT UR5e)
        % =========================================================
        fprintf('>>> TURNO DE P2 (BOT / ROBOT UR5e) <<<\n');
        
        fromTile = g.board.index(g.board.players(:, 2) == 1);
        cashP2 = g.assets.P2(g.assets.asset == Resource.cash);
        
        % El Bot toma su turno mediante la IA
        g = game.turnManager(g, model, epsilon);
        
        toTile = g.board.index(g.board.players(:, 2) == 1);
        cashP2Post = g.assets.P2(g.assets.asset == Resource.cash);
        netP2Post = g.assets.P2(g.assets.asset == Resource.netWorth);
        
        fprintf('El Bot se ha movido de la Casilla %d -> %d\n', fromTile, toTile);
        fprintf('Estado del Bot: Efectivo = $%d | Valor Neto = $%d\n', cashP2Post, netP2Post);
        
        % Si el robot está conectado, transmitir el movimiento por Socket TCP
        if robotConnected
            dataStruct = struct();
            dataStruct.turn = t;
            dataStruct.player = 2;
            dataStruct.from_tile = double(fromTile);
            dataStruct.to_tile = double(toTile);
            dataStruct.cash = double(cashP2Post);
            dataStruct.net_worth = double(netP2Post);
            dataStruct.is_jailed = logical(g.isJailed(2));
            
            jsonStr = jsonencode(dataStruct);
            fprintf('Transmitiendo movimiento del Bot al brazo robotico UR5e... ');
            writeline(client, jsonStr);
            
            respStr = readline(client);
            response = jsondecode(respStr);
            if strcmp(response.status, 'DONE')
                fprintf('[OK: El brazo UR5e movió la ficha en el tablero real]\n');
            end
        end
    end
    
    % Comprobar bancarrota
    if sum(g.isBankrupt) > 0
        fprintf('\n===================================================\n');
        fprintf('       ¡PARTIDA FINALIZADA POR BANCARROTA!        \n');
        fprintf('===================================================\n');
        break;
    end
end

if robotConnected
    clear client;
end

fprintf('\nFin de la partida interactiva.\n');
