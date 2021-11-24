%% Test af adc oploesning
clearvars
tic

%signaler
f1 = [280.1e6 281.1e6 281.9e6 286.6e6 288.2e6 289.0e6 290.2e6 290.3e6 292.8e6 294.5e6 295.2e6 295.7e6 296.5e6 297.0e6 298.4e6 299.32e6];
theta_a = pi; 
f2 = [280.1e6 281.1e6 281.9e6 286.6e6 288.2e6 289.0e6 290.2e6 290.3e6 292.8e6 294.5e6 295.2e6 295.7e6 296.5e6 297.0e6 298.4e6 299.32e6];
theta_b = pi/2;

Clambda = 0.1228658;
vOensket = (Clambda * 90)/(360 * 0.75 *Clambda);
droneVinkelOensket = (asin(vOensket)) * 180 / pi;


fs = 41.1e6;
sample_range = [256 256 2048]; %antal samples vi tester?? 
snr_range = [46 1 49]; %hvad tester vi af SNR f
b_range = [12 2 12];      %hvad bits tester vi med

antalSample = round((sample_range(3) - sample_range(1)) / sample_range(2));
antalB = round((b_range(3) - b_range(1)) / b_range(2));
antalSNR = round((snr_range(3) - snr_range(1)) / snr_range(2));


filterDC = 32;
samples = 2^17;
Tmax = samples/fs;
t = 0:1/fs:Tmax-1/fs;


%for fftB = 9:1:10
N = samples;           % oploesning af FFT
f = (-N/2:N/2-1)/N*fs;


difResult = zeros(antalB,antalSNR);
faseResult = zeros(antalB,antalSNR);
droneResult = zeros(antalB,antalSNR);

%for i_b = b_range(1):b_range(2):b_range(3)
%for i_e = snr_range(1):snr_range(2):snr_range(3)
e_range = [0.9:0.05:0.9];
    
Bits = 12;          % Bit number.    

    
for run_E = 1:1:length(e_range)
    eInput = e_range(1) - 0.05 + (run_E * 0.05);       % % af input range vi bruger
    %Signaler
    a = signalGen(f1,theta_a,t);
    b = signalGen(f1,theta_b,t);%cos((2*pi*300e6*t)+theta_b)+cos((2*pi*301e6*t)+theta_b)+cos((2*pi*289e6*t)+theta_b)+cos((2*pi*299e6*t)+theta_b)+cos((2*pi*293.6e6*t)+theta_b);

    aSampled = eInput*rescale(a,-1,1);
    bSampled = eInput*rescale(b,-1,1); 
    
    % ADC: Quantizer
    % Quantizing (abs of input value not over 1)
    partition = [-1:(2)/(2^Bits):1-(2)/(2^Bits)];
    codebook = [-1:(2)/(2^Bits):1-(2)/(2^Bits)];
    aDigital = quantiz(aSampled,partition,codebook); 
    bDigital = quantiz(bSampled,partition,codebook);    
    
    
    %fft stuff
    A = fft(aDigital,N);
    A = fftshift(A);
    B = fft(bDigital,N);
    B = fftshift(B);
    
    %sorterer DC tillaegget fra ADC'erne fra:
    for i = 1:1:(N/2)+(N/filterDC)   %(N/2)-(N/filterDC)
        A(i) = 0;
        B(i) = 0;
    end
    
    difAntal = 12;
    fase = zeros(1,difAntal);
    index = zeros(1,difAntal);
    faseA = zeros(1,difAntal);
    faseB = zeros(1,difAntal);
    for i = 1:1:difAntal
        [maxAmpValB, index(i)] = max(abs(B));
        faseB(i) = angle(B(index(i))) * 180 / pi;
        faseA(i) = angle(A(index(i))) * 180 / pi;
        B(index(i)) = 0;
        A(index(i)) = 0;
        fase(i) = mod(abs(faseB(i) - faseA(i)),180);
    end
     
        


    faseDif(run_E) = mean(fase,"all");
    v(run_E) = (Clambda * faseDif(run_E))/(360 * 0.75 *Clambda);
    droneVinkel(run_E) = (asin(v(run_E))) * 180 / pi
    difference(run_E) = droneVinkelOensket - droneVinkel(run_E);
    
end
% faseDif(runI) = mod(abs(faseB - faseA),180);
% v(runI) = (Clambda * faseDif(runI))/(360 * 0.75 *Clambda);
% droneVinkel(runI) = (asin(v(runI))) * 180 / pi;
% difference(runI) = droneVinkelOensket - droneVinkel(runI);
%     
% difference = abs(difference);
% difResult(taeller_b,taellerSamp) = mean(difference,'all');
% faseResult(taeller_b,taellerSamp) = mean(faseDif,'all');
% droneResult(taeller_b,taellerSamp) = mean(droneVinkel,'all');
                
                
%filename = "C:\Users\nikol\Desktop\resultater\" + N + "N.mat";
%mySave(filename,difResult,faseResult,droneResult);


toc

function signal = signalGen(freqs, faseShift, t)
    holder = 0;
    for i = 1:1:length(freqs)
        holder = holder + cos((2*pi*freqs(i)*t)+faseShift);
    end
    signal = holder;
end

function mySave(filenm, WT1, WT2, WT3)
    save(filenm, 'WT1', 'WT2', 'WT3');
end