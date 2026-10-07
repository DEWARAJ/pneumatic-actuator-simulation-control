function y=pneumatic_initial(p)
aa=pi*p.bore^2/4; ab=pi*(p.bore^2-p.rod^2)/4;
va=p.dead_volume+aa*p.initial_position;
vb=p.dead_volume+ab*(p.stroke-p.initial_position);
y=[p.initial_position;0;p.atmosphere*va/(p.R*p.temperature);...
    p.atmosphere*vb/(p.R*p.temperature);0;0;0;0];
end
