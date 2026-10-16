
target_speed=[0 80];
impact_time = 10;
impact_duration = 0.5;

duration = impact_time + (0:0.001:impact_duration);
fx_impact.time = duration';
fy_impact.time = duration';
fz_impact.time = duration';
x_impact.time = 0;
y_impact.time = 0;
z_impact.time = 0;

i = 0.5*(sin(2*pi/impact_duration*(duration-impact_time)-0.5*pi)+1);

fx_impact.signals.values = 5400*i';
fy_impact.signals.values = 14000*i';
fz_impact.signals.values = 0*i';
x_impact.signals.values = -2.835; 
y_impact.signals.values = -0.8;
z_impact.signals.values = 0;
