% intersectionmultipolicy_sumo.m
% Upgraded five-policy V2V offloading comparison using SUMO mobility traces.
% Run from project root: >> intersectionmultipolicy_sumo

clear; clc; close all;

% ── Parameters ────────────────────────────────────────────────────────────────
BANDWIDTH         = 50;      % MB/s
TASK_ARRIVAL_RATE = 0.5;     % tasks/s per vehicle
FCD_CSV           = 'src/sumo/fcd_traces.csv';

% ── Load SUMO traces ──────────────────────────────────────────────────────────
addpath('src');
[vehicles, timeSteps] = parse_fcd(FCD_CSV);
N_total = numel(vehicles);
fprintf('Loaded %d vehicles from SUMO traces.\n\n', N_total);

% ── Policy definitions ────────────────────────────────────────────────────────
policies = {@greedy_policy, @speedcheck_policy, @threshold_policy, ...
            @gametheoretic_policy, @loadbalance_policy};
names    = {'Greedy','SpeedCheck','Threshold','GameTheoretic','LoadBalancing'};
nP       = numel(policies);
rates    = zeros(nP, 2);

% ── Run each policy ───────────────────────────────────────────────────────────
fprintf('%-16s  %16s  %15s\n', 'Policy', 'Completion Rate', 'Offloading Rate');
fprintf('%s\n', repmat('-', 1, 52));

for pi = 1:nP
  fprintf('  Running %-14s ...', names{pi});
  [c, o, l, f] = run_sumo_sim(policies{pi}, vehicles, timeSteps, ...
                               BANDWIDTH, TASK_ARRIVAL_RATE);
  rates(pi,1) = c / max(1, c+f);
  rates(pi,2) = o / max(1, o+l);
  fprintf('  done\n');
  fprintf('  %-16s  %16.4f  %15.4f\n', names{pi}, rates(pi,1), rates(pi,2));
end

fprintf('\n');

% ── Bar chart ────────────────────────────────────────────────────────────────
figure('Name','SUMO Policy Comparison','Position',[300 200 700 450]);
bar(rates);
set(gca,'XTick',1:nP,'XTickLabel',names,'FontSize',10);
legend('Completion Rate','Offloading Rate','Location','northwest');
ylabel('Rate');
title('V2V Offloading Heuristics — SUMO Mobility Traces');
grid on;