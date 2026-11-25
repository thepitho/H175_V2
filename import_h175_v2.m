% Run with Simscape Multibody installed. URDF import; no controller included.
root = fileparts(mfilename('fullpath'));
old = pwd; cleanup = onCleanup(@() cd(old)); cd(root);
smimport(fullfile(root,'H175_V2.urdf'),'ModelName','H175_V2_multibody');
% Inspect all joint axes, mesh scale, joint limits and inertias before simulation.
% A floating base and ground contact must be added for walking studies.
