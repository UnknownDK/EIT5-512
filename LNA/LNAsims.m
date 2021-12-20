% Creating opamp based on scattering parameters.
unmatched_amp = read(rfckt.amplifier, 'BFP450rigtig.s2p');

%Analyzing the scattering parameters frequency range 1.4GHz - 5GHz.
analyze(unmatched_amp, 1.4e9:10e6:5e9);

%Plotting Gmag, Gavailable and transducer gain in dB.
figure
plot(unmatched_amp,'Gmag','Ga','Gt','dB')

%Plotting minimum noise and noise figure. 
plot(unmatched_amp,'Fmin','NF','dB')
axis([1.8 4 1 4])
legend('Location','NorthWest')

% Plotting the gain and noise circles in the smith chart.
fc = 2.441e9;
hsm = smithplot;
circle(unmatched_amp,fc,'Ga',10:1:13.16, ...
    'NF',1.82:0.2:3,hsm);
legend('Location','SouthEast')

%%
%Input matching values
Cin = 1.753e-12;
Lin = 1.993e-9

%Output matching values
Cout = 1.948e-12
Lout = 2.291e-9


%Creating input matching object consisting of a shunt inductor and a series capacitor.
input_match = rfckt.cascade('Ckts', ...
    {rfckt.shuntrlc('L',Lin),rfckt.seriesrlc('C',Cin)}); %Inserting component values.

%Creating output matching object consisting of a shunt inductor and a series capacitor.
output_match = rfckt.cascade('Ckts', ...
    {rfckt.seriesrlc('C',Cout),rfckt.shuntrlc('L',Lout)}); %Inserting component values.

LNA = rfckt.cascade('Ckts', ...
    {input_match,unmatched_amp,output_match});


%Analyzing the created LNA object with matching in the frequency spectrum
%2.3GHz to 2.5GHz
analyze(LNA,2.3e9:10e6:2.5e9);

%Plotting the gain, noisefigure in dB of for the LNA.
plot(LNA,'Gp', 'NF', 'dB');

