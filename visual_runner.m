% visual_runner.m
% Unified visual workflow: loads SUMO traces once, produces all four figure
% windows, and keeps them alive until the user closes them.
% Launched automatically by: make sumo-gui

clear; clc;
addpath('src');

fprintf('\n╔══════════════════════════════════════════╗\n');
fprintf('║   V2V Offloading — Visual Workflow        ║\n');
fprintf('╚══════════════════════════════════════════╝\n\n');

% ── Parameters ────────────────────────────────────────────────────────────────
BANDWIDTH         = 50;      % MB/s
TASK_ARRIVAL_RATE = 0.5;     % tasks/s per vehicle
N_TRIALS          = 10;      % set to 30 for paper-quality stats
FCD_CSV           = 'src/sumo/fcd_traces.csv';

% ── Load SUMO traces (once) ───────────────────────────────────────────────────
fprintf('[1/4] Loading SUMO traces...\n');
[vehicles, timeSteps] = parse_fcd(FCD_CSV);
N_total = numel(vehicles);
fprintf('      %d vehicles, %d timesteps\n\n', N_total, numel(timeSteps));

policies   = {@greedy_policy, @speedcheck_policy, @threshold_policy, ...
              @gametheoretic_policy, @loadbalance_policy};
names      = {'Greedy','SpeedCheck','Threshold','GameTheoretic','LoadBalancing'};
shortnames = {'Greedy','SpeedChk','Thresh','GameTh','LoadBal'};
lcolors    = {'b','g','r','m','k'};
nP         = numel(policies);

% ══════════════════════════════════════════════════════════════════════════════
% Figure 1 — Single-run bar chart
% ══════════════════════════════════════════════════════════════════════════════
fprintf('[1/4] Figure 1: single-run policy comparison...\n');
rates = zeros(nP, 2);
for pi = 1:nP
    [c, o, l, f] = run_sumo_sim(policies{pi}, vehicles, timeSteps, ...
                                 BANDWIDTH, TASK_ARRIVAL_RATE);
    rates(pi,1) = c / max(1, c+f);
    rates(pi,2) = o / max(1, o+l);
    fprintf('      %-16s  CR=%.4f  OR=%.4f\n', names{pi}, rates(pi,1), rates(pi,2));
end

f1 = figure('Name','Policy Comparison — Single Run','Position',[40 520 720 430]);
  b = bar(rates);
  b(1).FaceColor = [0.30 0.45 0.69]; b(2).FaceColor = [0.87 0.52 0.26];
  set(gca,'XTick',1:nP,'XTickLabel',names,'FontSize',10);
  xtickangle(22);
  legend('Completion Rate','Offloading Rate','Location','northwest','FontSize',10);
  ylabel('Rate','FontSize',11);
  title('V2V Offloading Policies — SUMO Mobility Traces','FontSize',12,'FontWeight','bold');
  grid on; ylim([0 1.12]);

% ══════════════════════════════════════════════════════════════════════════════
% Figure 2 — Statistical (N_TRIALS trials, error bars)
% ══════════════════════════════════════════════════════════════════════════════
fprintf('\n[2/4] Figure 2: statistical evaluation (%d trials)...\n', N_TRIALS);
all_rates = zeros(nP, N_TRIALS, 2);
for pi = 1:nP
    for t = 1:N_TRIALS
        rng(t * 100 + pi);
        [c,o,l,f] = run_sumo_sim(policies{pi}, vehicles, timeSteps, ...
                                  BANDWIDTH, TASK_ARRIVAL_RATE);
        all_rates(pi,t,1) = c / max(1,c+f);
        all_rates(pi,t,2) = o / max(1,o+l);
    end
    fprintf('      %-16s  done\n', names{pi});
end

cr_means = squeeze(mean(all_rates(:,:,1), 2));
cr_stds  = squeeze(std(all_rates(:,:,1),  0, 2));
or_means = squeeze(mean(all_rates(:,:,2), 2));
or_stds  = squeeze(std(all_rates(:,:,2),  0, 2));

f2 = figure('Name','Statistical Comparison','Position',[770 520 840 430]);
  subplot(1,2,1);
    bar(1:nP, cr_means); hold on;
    errorbar(1:nP, cr_means, cr_stds, 'k.', 'LineWidth', 1.8);
    set(gca,'XTickLabel',names,'XTick',1:nP,'FontSize',9); xtickangle(22);
    ylabel('Completion Rate'); ylim([0 1.12]);
    title(sprintf('Completion Rate  (n=%d trials)',N_TRIALS),'FontWeight','bold');
    grid on;
  subplot(1,2,2);
    bar(1:nP, or_means); hold on;
    errorbar(1:nP, or_means, or_stds, 'k.', 'LineWidth', 1.8);
    set(gca,'XTickLabel',names,'XTick',1:nP,'FontSize',9); xtickangle(22);
    ylabel('Offloading Rate');
    title(sprintf('Offloading Rate  (n=%d trials)',N_TRIALS),'FontWeight','bold');
    grid on;

% ══════════════════════════════════════════════════════════════════════════════
% Figure 3 — Density sweep
% ══════════════════════════════════════════════════════════════════════════════
fprintf('\n[3/4] Figure 3: vehicle density sweep...\n');
vehicle_counts  = [4, 8, 12, 16, min(20, N_total)];
density_results = zeros(nP, numel(vehicle_counts), 2);

for vi = 1:numel(vehicle_counts)
    nv      = vehicle_counts(vi);
    veh_sub = vehicles(1:min(nv, N_total));
    for pi = 1:nP
        rng(42);
        [c,o,l,flt] = run_sumo_sim(policies{pi}, veh_sub, timeSteps, 50, 0.5);
        density_results(pi,vi,1) = c / max(1,c+flt);
        density_results(pi,vi,2) = o / max(1,o+l);
    end
    fprintf('      N=%2d vehicles done\n', nv);
end

f3 = figure('Name','Density Sweep','Position',[40 60 920 400]);
  subplot(1,2,1); hold on;
    for pi = 1:nP
        plot(vehicle_counts, squeeze(density_results(pi,:,1)), ...
             '-o','Color',lcolors{pi},'LineWidth',1.8,'DisplayName',shortnames{pi});
    end
    xlabel('Number of Vehicles'); ylabel('Completion Rate');
    title('Density \rightarrow Completion Rate','FontWeight','bold');
    legend('Location','southwest'); grid on; ylim([0 1.05]);
  subplot(1,2,2); hold on;
    for pi = 1:nP
        plot(vehicle_counts, squeeze(density_results(pi,:,2)), ...
             '-s','Color',lcolors{pi},'LineWidth',1.8,'DisplayName',shortnames{pi});
    end
    xlabel('Number of Vehicles'); ylabel('Offloading Rate');
    title('Density \rightarrow Offloading Rate','FontWeight','bold');
    legend('Location','northwest'); grid on;

% ══════════════════════════════════════════════════════════════════════════════
% Figure 4 — Task load sweep
% ══════════════════════════════════════════════════════════════════════════════
fprintf('\n[4/4] Figure 4: task arrival rate sweep...\n');
arrival_rates = [0.25, 0.5, 1.0, 1.5, 2.0];
load_results  = zeros(nP, numel(arrival_rates), 2);

for ri = 1:numel(arrival_rates)
    for pi = 1:nP
        rng(42);
        [c,o,l,flt] = run_sumo_sim(policies{pi}, vehicles, timeSteps, 50, arrival_rates(ri));
        load_results(pi,ri,1) = c / max(1,c+flt);
        load_results(pi,ri,2) = o / max(1,o+l);
    end
    fprintf('      rate=%.2f tasks/s done\n', arrival_rates(ri));
end

f4 = figure('Name','Load Sweep','Position',[970 60 920 400]);
  subplot(1,2,1); hold on;
    for pi = 1:nP
        plot(arrival_rates, squeeze(load_results(pi,:,1)), ...
             '-^','Color',lcolors{pi},'LineWidth',1.8,'DisplayName',shortnames{pi});
    end
    xlabel('Task Arrival Rate (tasks/s)'); ylabel('Completion Rate');
    title('Load \rightarrow Completion Rate','FontWeight','bold');
    legend('Location','southwest'); grid on;
  subplot(1,2,2); hold on;
    for pi = 1:nP
        plot(arrival_rates, squeeze(load_results(pi,:,2)), ...
             '-d','Color',lcolors{pi},'LineWidth',1.8,'DisplayName',shortnames{pi});
    end
    xlabel('Task Arrival Rate (tasks/s)'); ylabel('Offloading Rate');
    title('Load \rightarrow Offloading Rate','FontWeight','bold');
    legend('Location','northwest'); grid on;

% ══════════════════════════════════════════════════════════════════════════════
fprintf('\n╔══════════════════════════════════════════╗\n');
fprintf('║  All 4 figures open. Close them to exit. ║\n');
fprintf('╚══════════════════════════════════════════╝\n');

% Keep alive until every figure window is closed
h = findall(0, 'Type', 'figure');
arrayfun(@waitfor, h);