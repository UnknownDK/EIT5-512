%% Input
f       = 2.441e9;          % Center frequency, Hz
Zin     = 13.604 + 14.940*1i;      % Gamma optimum, Ohms
Zl      = 50;       % Load impedance, Ohms
Zo      = 50;           % characteristic impedance of system, Ohms

%Defining the circuits . 
n = matchingnetwork('CenterFrequency',f,'Bandwidth',1e8,           ...
  'LoadImpedance',Zl,'SourceImpedance',Zin,'Components',2);
%Defining circuit. 
n.Circuit(2)
%Plotting the matching network in the smith chart. 
hs = smithplot(n, 'Z0', Zo, 'CircuitIndex',2);


%% Output
f       = 2.441e9;          % Center frequency, Hz
Zin     = 16.53 + 9.95*1i;      % S22, Ohms
Zl      = 50;       % Load impedance, Ohms
Zo      = 50;           % characteristic impedance of system, Ohms

%Defining the circuits . 
n = matchingnetwork('CenterFrequency',f,'Bandwidth',1e8,           ...
  'LoadImpedance',Zl,'SourceImpedance',Zin,'Components',2);
%Defining LC or CL circuit by chosing 1 or 2. 
n.Circuit(2)
%Plotting the matching network in the smith chart. 
hs = smithplot(n, 'Z0', Zo, 'CircuitIndex',2);