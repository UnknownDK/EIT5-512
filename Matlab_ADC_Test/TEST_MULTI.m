%% Test af adc oploesning
clearvars
tic


Clambda = 0.1228658;
vOensket = (Clambda * 90)/(360 * 0.75 *Clambda);
droneVinkelOensket = (asin(vOensket)) * 180 / pi;


fs = 87.1e2;
snr_range = [10];%20 30 40 50 60 70 80]; %hvad tester vi af SNR f
b_range = [12 2 12];      %hvad bits tester vi med
e_range = [0.9:0.05:0.9];

antalB = round((b_range(3) - b_range(1)) / b_range(2));
antalSNR = length(snr_range);
antalTests = 1;

filterDC = 128;
samples = 2^20;
Tmax = (samples/fs)/2;%samples/fs;
t = -Tmax:1/fs:Tmax-1/fs;


%signaler
fh = 300e2;
fl = 280e2;
theta_a = -pi/4; 
theta_b = pi/4;
ting = 2*fh*sinc(2*fh*t);
a = (2*fh*sinc(2*fh*(t+theta_a))) - (2*fl*sinc(2*fl*(t+theta_a)));
b = (2*fh*sinc(2*fh*(t+theta_b))) - (2*fl*sinc(2*fl*(t+theta_b)));

%for fftB = 9:1:10
N = samples;           % oploesning af FFT
f = (-N/2:N/2-1)/N*fs;


difResult = zeros(1,antalSNR);
faseResult = zeros(antalB,antalSNR);
droneResult = zeros(antalB,antalSNR);


%for i_b = b_range(1):b_range(2):b_range(3)
%

    
Bits = 12;          % Bit number.    
eInput = 0.9;


aSampled = eInput*rescale(a,-1,1);
bSampled = eInput*rescale(b,-1,1); 

for run_SNR = 1:1:antalSNR
    snrLevel = 120;%snr_range(1) - 10 + (run_SNR * 10);
    
    faseDif = zeros(antalTests,1);
    v = zeros(antalTests,1);
    droneVinkel = zeros(antalTests,1);
    difference = zeros(antalTests,1);
    for runI = 1:antalTests
    
        aNoise = awgn(aSampled,snrLevel,'measured');
        bNoise = awgn(bSampled,snrLevel,'measured');
    
        % ADC: Quantizer
        % Quantizing (abs of input value not over 1)
        partition = [-1:(2)/(2^Bits):1-(2)/(2^Bits)];
        codebook = [-1:(2)/(2^Bits):1-(2)/(2^Bits)];
        aDigital = quantiz(aNoise,partition,codebook); 
        bDigital = quantiz(bNoise,partition,codebook);    


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

        difAntal = 7000;
        fase = zeros(1,difAntal);
        index = zeros(1,difAntal);
        faseA = zeros(1,difAntal);
        faseB = zeros(1,difAntal);
        for i = 1:1:difAntal
            %[maxAmpValB, index(i)] = max(abs(B));
            index(i) = 9.45e+05 + i*3;
            faseB(i) = angle(B(index(i))) * 180 / pi;
            faseA(i) = angle(A(index(i))) * 180 / pi;
            B(index(i)) = 0;
            A(index(i)) = 0;
            fase(i) = mod(abs(faseA(i) - faseB(i)),180);
        end


        faseDif(runI) = mean(fase,"all");
        v(runI) = (Clambda * faseDif(runI))/(360 * 0.75 *Clambda);
        droneVinkel(runI) = (asin(v(runI))) * 180 / pi;
        difference(runI) = droneVinkelOensket - droneVinkel(runI);
    

    end
    difference = abs(difference);
    difResult(run_SNR) = mean(difference,'all');
    %faseResult(b_taeller,snr_taeller) = mean(faseDif,'all');
    %droneResult(b_taeller,snr_taeller) = mean(droneVinkel,'all');
    
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