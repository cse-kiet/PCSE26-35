% run_trials.m
% Statistical evaluation: 30 trials per policy, report mean +/- std.
% Run from project root: >> run_trials

clear; clc; close all;

N_TRIALS          = 30;
BANDWIDTH         = 50;
TASK_ARRIVAL_RATE = 0.5;
FCD_CSV           = 'src/sumo/fcd_traces.csv';

addpath('src');
[vehicles, timeSteps] = parse_fcd(FCD_CSV);

policies = {@greedy_policy, @speedcheck_policy, @threshold_policy, ...
            @gametheoretic_policy, @loadbalance_policy};
names    = {'Greedy','SpeedCheck','Threshold','GameTheoretic','LoadBalancing'};
nP       = numel(policies);

% Storage: nP x N_TRIALS x 2 (completion, offloading)
all_rates = zeros(nP, N_TRIALS, 2);

fprintf('Running %d trials per policy...\n\n', N_TRIALS);

for pi = 1:nP
  fprintf('Policy: %-16s  [', names{pi});
  for trial = 1:N_TRIALS
    rng(trial * 100 + pi);
    [c, o, l, f] = run_sumo_sim(policies{pi}, vehicles, timeSteps, ...
                                 BANDWIDTH, TASK_ARRIVAL_RATE);
    all_rates(pi, trial, 1) = c / max(1, c+f);
    all_rates(pi, trial, 2) = o / max(1, o+l);
    if mod(trial, 5) == 0, fprintf('.'); end
  end
  fprintf(']\n');
end

fprintf('\n');
fprintf('%-16s  %22s  %22s\n', 'Policy', ...
        'Completion (mean+/-std)', 'Offloading (mean+/-std)');
fprintf('%s\n', repmat('-', 1, 65));

for pi = 1:nP
  cr_mean = mean(all_rates(pi,:,1));
  cr_std  = std(all_rates(pi,:,1));
  or_mean = mean(all_rates(pi,:,2));
  or_std  = std(all_rates(pi,:,2));
  fprintf('%-16s  %10.4f +/- %6.4f    %10.4f +/- %6.4f\n', ...
          names{pi}, cr_mean, cr_std, or_mean, or_std);
end

% ── Error bar plot ────────────────────────────────────────────────────────────
figure('Name','Statistical Comparison','Position',[300 200 800 450]);
cr_means = squeeze(mean(all_rates(:,:,1), 2));
cr_stds  = squeeze(std(all_rates(:,:,1),  0, 2));
or_means = squeeze(mean(all_rates(:,:,2), 2));
or_stds  = squeeze(std(all_rates(:,:,2),  0, 2));

x = 1:nP;
subplot(1,2,1);
  bar(x, cr_means); hold on;
  errorbar(x, cr_means, cr_stds, 'k.', 'LineWidth', 1.5);
  set(gca,'XTickLabel', names, 'XTick', x, 'FontSize', 9);
  xtickangle(20); ylabel('Completion Rate');
  title(sprintf('Completion Rate (N=%d trials)', N_TRIALS)); grid on;

subplot(1,2,2);
  bar(x, or_means); hold on;
  errorbar(x, or_means, or_stds, 'k.', 'LineWidth', 1.5);
  set(gca,'XTickLabel', names, 'XTick', x, 'FontSize', 9);
  xtickangle(20); ylabel('Offloading Rate');
  title(sprintf('Offloading Rate (N=%d trials)', N_TRIALS)); grid on;