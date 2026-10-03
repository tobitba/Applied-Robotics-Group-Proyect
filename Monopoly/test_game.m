% TEST_GAME.M - Script simple para probar el motor de Monopoly y el Bot entrenado
% Para ejecutar en MATLAB: escribe "test_game" en la ventana de comandos.

clc;
clear;
close all;

% Agregar carpetas del repositorio al PATH de MATLAB
addpath('Classes');
addpath(genpath('.'));

fprintf('===================================================\n');
fprintf('       PROBANDO MOTOR DE MONOPOLY (BOT-OPOLY)     \n');
fprintf('===================================================\n\n');

% 1. Cargar el modelo pre-entrenado
modelPath = fullfile('Resources', 'Models', 'model.mat');
fprintf('1. Cargando el modelo pre-entrenado desde: %s ...\n', modelPath);

if exist(modelPath, 'file')
    loaded = load(modelPath);
    model = loaded.model;
    fprintf('   -> Modelo cargado exitosamente!\n\n');
else
    error('No se encontro el archivo model.mat en Resources/Models/');
end

% 2. Inicializar una partida con 4 jugadores
numPlayers = 4;
fprintf('2. Inicializando partida con %d jugadores...\n', numPlayers);
g = Monopoly(numPlayers);

% 3. Simular 20 turnos mostrando las acciones en consola
fprintf('3. Simulando 20 turnos del Bot contra si mismo...\n');
fprintf('---------------------------------------------------\n');

epsilon = zeros(1, 4); % 0 = El bot usa 100% su modelo entrenado (modo optimo)

for t = 1:20
    jugadorActual = g.current;
    cashAntes = g.assets.("P" + string(jugadorActual))(g.assets.asset == Resource.cash);
    
    % Ejecutar turno usando turnManager
    g = game.turnManager(g, model, epsilon);
    
    cashDespues = g.assets.("P" + string(jugadorActual))(g.assets.asset == Resource.cash);
    netWorth = g.assets.("P" + string(jugadorActual))(g.assets.asset == Resource.netWorth);
    
    fprintf('Turno %2d | Jugador P%d | Efectivo: $%d | Valor Neto: $%d\n', ...
        t, jugadorActual, cashDespues, netWorth);
end

fprintf('---------------------------------------------------\n');
fprintf('4. Estado de los activos finales:\n\n');
disp(g.assets);

fprintf('\n5. Resumen de propiedades compradas en el tablero:\n');
propiedadesCompradas = g.board(g.board.isOwned == true, {'index', 'owner', 'numHouses', 'isMortgaged'});
disp(propiedadesCompradas);

fprintf('===================================================\n');
fprintf('   ¡PRUEBA FINALIZADA CON EXITO!                  \n');
fprintf('===================================================\n');
