scale = 3;

fh = 30 * 10^scale;
fl = 28 * 10^scale;

fs = 7.79 * 10^scale;
samples = 2^15;
Tmax = (samples/fs)/2;%samples/fs;
t = -Tmax:1/fs:Tmax-1/fs;
N = samples;
f = (-N/2:N/2-1)/N*fs;


theta_a = -pi/2; 
theta_b = pi/2;
t2 = t + theta_b;
a = (2*fh*sinc(2*fh*(t+theta_a))) - (2*fl*sinc(2*fl*(t+theta_a)));
b = (2*fh*sinc((2*fh*t2))) - (2*fl*sinc((2*fl*t2)));


% faseB = pi;
% a = (2*fh*sinc(2*fh*t)) - (2*fl*sinc(2*fl*t));
% b = (2*fh*sinc(2*fh*(t+faseB))) - (2*fl*sinc(2*fl*(t+faseB)));



A = fft(a,N);
A = fftshift(A);
B = fft(b,N);
B = fftshift(B);


%plottet signalerne A og B i tidsdomaene
tiledlayout(2,2)
nexttile
stem(f,abs(A))
title('Undersampled signal a')
xlabel 'Frequency (Hz)'
ylabel '|y|'
%plot(t,a)
%hold on
%plot(t,b)
%plot(t,(a-b))
%hold off

%Plotter styrker af A og B
nexttile
%stem(f,abs(A))
% hold on
stem(f,abs(B))
title('Undersampled signal b')
xlabel 'Frequency (Hz)'
ylabel '|y|'
grid
% hold off

%fjerner stoej så vi ikke plotter vinklen af støj
tol = 1.0e+1;
A(abs(A) < tol) = 0;
B(abs(B) < tol) = 0;

%plotter vinklen a A
nexttile
thetaA = angle(A);
stem(f,thetaA/pi)
title('Fase signal a')
xlabel 'Frequency (Hz)'
ylabel 'Phase / \pi'
grid
% hold on
% 
% %tilfoejer plottet a B oveni A
thetaB = angle(B);
% stem(f,thetaB/pi,'x')
% hold off

%Printer vinklen af B - A 
test = mod(abs(thetaB - thetaA),pi);
nexttile
stem(f,test/pi)
title('Faseforskel')
xlabel 'Frequency (Hz)'
ylabel 'Phase / \pi'
grid