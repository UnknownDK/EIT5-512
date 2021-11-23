%% Test af adc oploesning
clearvars
tic
parfor (fftB = 5:1:16)
    fs = 42e6;
    Tmax = 0.0005;
    t = 0:1/fs:Tmax-1/fs;

    %signaler
    f1 = 300e6;
    theta_a = pi; 
    f2 = 300e6;
    theta_b = pi/2;


    N = 2^fftB;           % oploesning af FFT
    f = (-N/2:N/2-1)/N*fs;
    X_m = 1;            % Range i volt er: +- X_m
    Clambda = 0.1228658;
    vOensket = (Clambda * 90)/(360 * 0.75 * Clambda);
    droneVinkelOensket = (asin(vOensket)) * 180 / pi;

    %downsample til ADC Tror ikke virker
    %digital_f = 80e2;
    %n = downsample(t,fs/digital_f);

    e_range = [1 0.05 1]; %hvad tester vi for af input procenter
    b_range = [6 2 16];      %hvad bits tester vi med
    antalB = round((b_range(3) - b_range(1)) / b_range(2));
    antalE = round((e_range(3) - e_range(1)) / e_range(2));

    difResult = zeros(antalB,antalE);
    faseResult = zeros(antalB,antalE);
    droneResult = zeros(antalB,antalE);

    taeller_b = 1;
    for i_b = b_range(1):b_range(2):b_range(3)

        taeller_e = 1;
        for i_e = e_range(1):e_range(2):e_range(3)

            Bits = i_b;            % Bit number.
            eInput = i_e;       % % af input range vi bruger

            %Signaler
            a = eInput*X_m*cos((2*pi*f1*t)+theta_a);
            b = eInput*X_m*cos((2*pi*f2*t)+theta_b);

            %k=(f1/fs)*2*pi/max(t); 
            %a = modulate(t,f1,fs,'fm',k);


            % ADC
            % Sampling
            aSampled = a;
            bSampled = b;

            % ADC: Quantizer
            % Quantizing (abs of input value should not over 1)
            partition = [-X_m:(2*X_m)/(2^Bits):X_m-(2*X_m)/(2^Bits)];
            codebook = [-X_m:(2*X_m)/(2^Bits):X_m-(2*X_m)/(2^Bits)];
            aDigital = quantiz(aSampled,partition,codebook); 
            bDigital = quantiz(bSampled,partition,codebook); 




            %fft stuff
            A = fft(aDigital,N);
            A = fftshift(A);
            B = fft(bDigital,N);
            B = fftshift(B);

            %sorterer DC tillaegget fra ADC'erne fra:
            for i = (N/2)-(N/16):1:(N/2)+(N/16)
                A(i) = 0;
                B(i) = 0;
            end



    %         %plottet signalerne A og B i tidsdomaene
    %         tiledlayout(2,2)
    %         nexttile
    %         plot(t,aDigital)
    %         hold on
    %         plot(t,bDigital)
    %         %plot(t,(a-b))
    %         hold off
    % 
    %         %Plotter styrker af A og B
    %         nexttile
    %         stem(f,abs(A))
    %         xlabel 'Frequency (Hz)'
    %         ylabel '|y|'
    %         grid
    %         hold on
    %         stem(f,abs(B))
    %         hold off
    % 
    %         %fjerner stoej så vi ikke plotter vinklen af støj
    %         tol = 1.0e+2;
    %         A(abs(A) < tol) = 0;
    %         B(abs(B) < tol) = 0;
    % 
    %         %plotter vinklen a A
    %         nexttile
             thetaA = angle(A);
    %         stem(f,thetaA/pi)
    %         xlabel 'Frequency (Hz)'
    %         ylabel 'Phase / \pi'
    %         grid
    %         hold on
    % 
    %         %tilfoejer plottet a B oveni A
             thetaB = angle(B);
    %         stem(f,thetaB/pi,'x')
    %         hold off
    % 
    %         %Printer vinklen af B - A 
    %         test = thetaB - thetaA;
    %         nexttile
    %         stem(f,test/pi)
    %         xlabel 'Frequency (Hz)'
    %         ylabel 'Phase / \pi'
    %         grid


            [maxAmpValB, indexB] = max(abs(B));
            [maxAmpValA, indexA]= max(abs(A));
            faseB = thetaB(indexB) * 180 / pi;
            faseA = thetaA(indexB) * 180 / pi;
 


            faseDif = mod(abs(faseB - faseA),180);
            v = (Clambda * faseDif)/(360 * 0.75 *Clambda);
            droneVinkel = (asin(v)) * 180 / pi;
            difference = droneVinkelOensket - droneVinkel;

            difResult(taeller_b,taeller_e) = difference;
            faseResult(taeller_b,taeller_e) = faseDif;
            droneResult(taeller_b,taeller_e) = droneVinkel;


            taeller_e = taeller_e +1;
        end
        taeller_b = taeller_b + 1;
    end

    filename = "C:\Users\Nikolaj\Desktop\resultater\" + N + "N.mat";
    mySave(filename,difResult,faseResult,droneResult);

end
toc

function mySave(filenm, WT1, WT2, WT3)
    save(filenm, 'WT1', 'WT2', 'WT3');
end