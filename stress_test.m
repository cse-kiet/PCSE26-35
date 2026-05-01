% stress_test.m
% Sweep vehicle density and task arrival rate across all five policies.
% Produces four subplot figures (density and load sweeps).
% Run from project root: >> stress_test

clear; clc; close all;

addpath('src');
FCD_CSV = 'src/sumo/fcd_traces.csv';
[vehicles_all, timeSteps] = parse_fcd(FCD_CSV);
N_total = numel(vehicles_all);

policies = {@greedy_policy, @speedcheck_policy, @threshold_policy, ...
            @gametheoretic_policy, @loadbalance_policy};
names    = {'Greedy','SpeedCheck','Threshold','GameTh','LoadBal'};
colors   = {'b','g','r','m','k'};
nP       = numel(policies);

% ── Sweep 1: Vehicle density ──────────────────────────────────────────────────
vehicle_counts  = [4, 8, 12, 16, min(20, N_total)];
density_results = zeros(nP, numel(vehicle_counts), 2);

fprintf('Density sweep: %s vehicles...\n', mat2str(vehicle_counts));
for vi = 1:numel(vehicle_counts)
  nv = vehicle_counts(vi);
  veh_subset = vehicles_all(1:min(nv, N_total));
  for pi = 1:nP
    rng(42);
    [c,o,l,f] = run_sumo_sim(policies{pi}, veh_subset, timeSteps, 50, 0.5);
    density_results(pi,vi,1) = c / max(1,c+f);
    density_results(pi,vi,2) = o / max(1,o+l);
  end
  fprintf('  N=%d done\n', nv);
end

% ── Sweep 2: Task arrival rate ────────────────────────────────────────────────
arrival_rates = [0.25, 0.5, 1.0, 1.5, 2.0];
load_results  = zeros(nP, numel(arrival_rates), 2);

fprintf('\nArrival rate sweep: %s tasks/s...\n', mat2str(arrival_rates));
for ri = 1:numel(arrival_rates)
  ar = arrival_rates(ri);
  for pi = 1:nP
    rng(42);
    [c,o,l,f] = run_sumo_sim(policies{pi}, vehicles_all, timeSteps, 50, ar);
    load_results(pi,ri,1) = c / max(1,c+f);
    load_results(pi,ri,2) = o / max(1,o+l);
  end
  fprintf('  rate=%.2f done\n', ar);
end

% ── Plot density sweep ────────────────────────────────────────────────────────
figure('Name','Density Sweep','Position',[100 100 900 400]);
subplot(1,2,1);
  hold on;
  for pi = 1:nP
    plot(vehicle_counts, squeeze(density_results(pi,:,1)), ...
         '-o', 'Color', colors{pi}, 'LineWidth', 1.8, 'DisplayName', names{pi});
  end
  xlabel('Number of Vehicles'); ylabel('Completion Rate');
  title('Effect of Vehicle Density on Completion Rate');
  legend('Location','southwest'); grid on; ylim([0 1.05]);

subplot(1,2,2);
  hold on;
  for pi = 1:nP
    plot(vehicle_counts, squeeze(density_results(pi,:,2)), ...
         '-s', 'Color', colors{pi}, 'LineWidth', 1.8, 'DisplayName', names{pi});
  end
  xlabel('Number of Vehicles'); ylabel('Offloading Rate');
  title('Effect of Vehicle Density on Offloading Rate');
  legend('Location','northwest'); grid on;

% ── Plot load sweep ───────────────────────────────────────────────────────────
figure('Name','Load Sweep','Position',[100 550 900 400]);
subplot(1,2,1);
  hold on;
  for pi = 1:nP
    plot(arrival_rates, squeeze(load_results(pi,:,1)), ...
         '-^', 'Color', colors{pi}, 'LineWidth', 1.8, 'DisplayName', names{pi});
  end
  xlabel('Task Arrival Rate (tasks/s)'); ylabel('Completion Rate');
  title('Effect of Task Load on Completion Rate');
  legend('Location','southwest'); grid on;

subplot(1,2,2);
  hold on;
  for pi = 1:nP
    plot(arrival_rates, squeeze(load_results(pi,:,2)), ...
         '-d', 'Color', colors{pi}, 'LineWidth', 1.8, 'DisplayName', names{pi});
  end
  xlabel('Task Arrival Rate (tasks/s)'); ylabel('Offloading Rate');
  title('Effect of Task Load on Offloading Rate');
  legend('Location','northwest'); grid on;

fprintf('\nStress test complete. Check figures.\n');