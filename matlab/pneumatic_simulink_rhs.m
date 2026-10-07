function dy=pneumatic_simulink_rhs(u)
% Simulation wrapper for the reference plant plus controller equations.
% This is not an Amesim co-simulation interface.
persistent p
if isempty(p), p=pneumatic_parameters('tuned'); end
dy=pneumatic_rhs(u(1),u(2:9),p);
end
