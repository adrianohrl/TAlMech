clc;
clear all;
close all;

% Gaussiana do Ciclo (1.5, 1.0)
desired_mean = 1.5;
desired_std = 1.0;
filename = 'analytics/cycles.csv';
data = csvread(filename, 1, 0)(2 : end - 1, 2);
tasks = length(data);
disp(['Mean: ' num2str(mean(data))]);
disp(['Standard deviation: ' num2str(std(data))]);
[counters centers] = hist(data);
counters = 100 * counters / sum(counters);
h = figure;
bar(centers, counters);
hold on;
x = linspace(centers(1), centers(end), 100);
fc = max(counters) / normpdf(desired_mean, desired_mean, desired_std);
plot(x, fc * normpdf(x, desired_mean, desired_std), '-r', 'LineWidth', 3);
fc = max(counters) / normpdf(mean(data), mean(data), std(data));
plot(x, fc * normpdf(x, mean(data), std(data)), '--k', 'LineWidth', 3);
grid on;
title('Histograma da duração entre a geração de tarefas');
ylabel('Frequência [%] ');
xlabel('Duração [s]');
%legend('Histograma', ['N~(' num2str(desired_mean) ',' num2str(desired_std) ')'], ['N~(' num2str(mean(data)) ',' num2str(std(data)) ')']);
%saveas(h, 'analytics/cycles.tex');
print -deps -color file.eps

% Gaussiana do Número de Waypoints (4, 2)
desired_mean = 4;
desired_std = 2;
filename = 'analytics/waypoints.csv';
data = csvread(filename, 1, 0)(2 : end - 1, 2);
disp(['Mean: ' num2str(mean(data))]);
disp(['Standard deviation: ' num2str(std(data))]);
[counters centers] = hist(data);
counters = 100 * counters / sum(counters);
h = figure;
bar(centers, counters);
hold on;
x = linspace(centers(1), centers(end), 100);
fc = max(counters) / normpdf(desired_mean, desired_mean, desired_std);
plot(x, fc * normpdf(x, desired_mean, desired_std), '-r', 'LineWidth', 3);
fc = max(counters) / normpdf(mean(data), mean(data), std(data));
plot(x, fc * normpdf(x, mean(data), std(data)), '--k', 'LineWidth', 3);
grid on;
title('Histograma do número de pontos de passagem gerados');
ylabel('Frequência [%] ');
xlabel('Quantidade [un]');
saveas(h, 'analytics/waypoints.tex');

% Gaussiana da Coordenada das abcissas (0.0, 10.0)
desired_mean = 0.0;
desired_std = 10.0;
filename = 'analytics/waypoints-x.csv';
data = csvread(filename, 1, 0)(2 : end - 1, 2);
disp(['Mean: ' num2str(mean(data))]);
disp(['Standard deviation: ' num2str(std(data))]);
[counters centers] = hist(data);
counters = 100 * counters / sum(counters);
h = figure;
bar(centers, counters);
hold on;
x = linspace(centers(1), centers(end), 100);
fc = max(counters) / normpdf(desired_mean, desired_mean, desired_std);
plot(x, fc * normpdf(x, desired_mean, desired_std), '-r', 'LineWidth', 3);
fc = max(counters) / normpdf(mean(data), mean(data), std(data));
plot(x, fc * normpdf(x, mean(data), std(data)), '--k', 'LineWidth', 3);
grid on;
title('Histograma da coordenada das abcissas');
ylabel('Frequência [%] ');
xlabel('Coordenada [m]');
saveas(h, 'analytics/waypoints-x.tex');

% Gaussiana da Coordenada das ordenadas (0.0, 15.0)
desired_mean = 0.0;
desired_std = 15.0;
filename = 'analytics/waypoints-y.csv';
data = csvread(filename, 1, 0)(2 : end - 1, 2);
disp(['Mean: ' num2str(mean(data))]);
disp(['Standard deviation: ' num2str(std(data))]);
[counters centers] = hist(data);
counters = 100 * counters / sum(counters);
h = figure;
bar(centers, counters);
hold on;
x = linspace(centers(1), centers(end), 100);
fc = max(counters) / normpdf(desired_mean, desired_mean, desired_std);
plot(x, fc * normpdf(x, desired_mean, desired_std), '-r', 'LineWidth', 3);
fc = max(counters) / normpdf(mean(data), mean(data), std(data));
plot(x, fc * normpdf(x, mean(data), std(data)), '--k', 'LineWidth', 3);
grid on;
title('Histograma da coordenada das ordenadas');
ylabel('Frequência [%] ');
xlabel('Coordenada [m]');
saveas(h, 'analytics/waypoints-y.tex');

% Probabilidade do skill 'camera' (0.75, unário)
filename = 'analytics/skills-camera.csv';
data = csvread(filename, 1, 0)(2 : end - 1, 2);
disp(['O recurso camera foi requisitado ' num2str(100 * length(data) / tasks) ' [%] das vezes que uma tarefa foi gerada.']);

% Probabilidade do skill 'battery' (0.7, contínuo)
filename = 'analytics/skills-battery.csv';
data = csvread(filename, 1, 0)(2 : end - 1, 2);
disp(['O recurso battery foi requisitado ' num2str(100 * length(data) / tasks) ' [%] das vezes que uma tarefa foi gerada.']);

% Gaussiana da quantidade necessária de bateria (0.6, 0.2)
desired_mean = 0.6;
desired_std = 0.2;
filename = 'analytics/skills-level-battery.csv';
data = csvread(filename, 1, 0)(2 : end - 1, 2);
disp(['Mean: ' num2str(mean(data))]);
disp(['Standard deviation: ' num2str(std(data))]);
[counters centers] = hist(data);
counters = 100 * counters / sum(counters);
h = figure;
bar(centers, counters);
hold on;
x = linspace(centers(1), centers(end), 100);
fc = max(counters) / normpdf(desired_mean, desired_mean, desired_std);
plot(x, fc * normpdf(x, desired_mean, desired_std), '-r', 'LineWidth', 3);
fc = max(counters) / normpdf(mean(data), mean(data), std(data));
plot(x, fc * normpdf(x, mean(data), std(data)), '--k', 'LineWidth', 3);
grid on;
title('Histograma da quantidade de bateria requerida');
ylabel('Frequência [%] ');
xlabel('Quantidade [%]');
saveas(h, 'analytics/skills-battery.tex');

% Probabilidade do skill 'processor' (0.6, discreto)
filename = 'analytics/skills-processor.csv';
data = csvread(filename, 1, 0)(2 : end - 1, 2);
disp(['O recurso processor foi requisitado ' num2str(100 * length(data) / tasks) ' [%] das vezes que uma tarefa foi gerada.']);

% Gaussiana da quantidade necessária de processador (3, 1)
desired_mean = 3;
desired_std = 1;
filename = 'analytics/skills-level-processor.csv';
data = csvread(filename, 1, 0)(2 : end - 1, 2);
disp(['Mean: ' num2str(mean(data))]);
disp(['Standard deviation: ' num2str(std(data))]);
centers = min(data) : max(data);
[counters centers] = hist(data, centers, length(centers));
counters = 100 * counters / sum(counters);
h = figure;
bar(centers, counters, 'hist');
hold on;
x = linspace(centers(1), centers(end), 100);
fc = max(counters) / normpdf(desired_mean, desired_mean, desired_std);
plot(x, fc * normpdf(x, desired_mean, desired_std), '-r', 'LineWidth', 3);
fc = max(counters) / normpdf(mean(data), mean(data), std(data));
plot(x, fc * normpdf(x, mean(data), std(data)), '--k', 'LineWidth', 3);
grid on;
title('Histograma da quantidade de processadores requerida');
ylabel('Frequência [%] ');
xlabel('Quantidade [un]');
saveas(h, 'analytics/skills-processor.tex');

% Probabilidade do skill 'strength' (0.5, contínua)
filename = 'analytics/skills-strength.csv';
data = csvread(filename, 1, 0)(2 : end - 1, 2);
disp(['O recurso strength foi requisitado ' num2str(100 * length(data) / tasks) ' [%] das vezes que uma tarefa foi gerada.']);

% Gaussiana da quantidade necessária de força (5.6, 3.4)
desired_mean = 5.6;
desired_std = 3.4;
filename = 'analytics/skills-level-strength.csv';
data = csvread(filename, 1, 0)(2 : end - 1, 2);
disp(['Mean: ' num2str(mean(data))]);
disp(['Standard deviation: ' num2str(std(data))]);
[counters centers] = hist(data);
counters = 100 * counters / sum(counters);
h = figure;
bar(centers, counters);
hold on;
x = linspace(centers(1), centers(end), 100);
fc = max(counters) / normpdf(desired_mean, desired_mean, desired_std);
plot(x, fc * normpdf(x, desired_mean, desired_std), '-r', 'LineWidth', 3);
fc = max(counters) / normpdf(mean(data), mean(data), std(data));
plot(x, fc * normpdf(x, mean(data), std(data)), '--k', 'LineWidth', 3);
grid on;
title('Histograma da intensidade de força requerida');
ylabel('Frequência [%] ');
xlabel('Intensidade [N]');
saveas(h, 'analytics/skills-strength.tex');

% Probabilidade do skill 'laserscan' (0.35, unário)
filename = 'analytics/skills-laserscan.csv';
data = csvread(filename, 1, 0)(2 : end - 1, 2);
disp(['O recurso laserscan foi requisitado ' num2str(100 * length(data) / tasks) ' [%] das vezes que uma tarefa foi gerada.']);