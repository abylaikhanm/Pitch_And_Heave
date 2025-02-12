%% Measure Angular Position with an Incremental Rotary Encoder


%% Create a Data Acquisition Object
clc; clear all; close all;
% Create a data acquisition object and add an input channel with |Position|
% measurement type.

A = 10; % Amplitude in degrees
f = 5; % Frequency in Hertz
IRcycles = 1;

s = daq('ni');
samplingRate = 1000;
s.Rate = samplingRate;
daqDuration = seconds(5);

clock = addinput(s,'Dev2','ai0','Voltage');

ch1 = addinput(s, 'Dev2', 'ctr0', 'Position');
ch1.EncoderType = 'X1';
ch2 = addinput(s, 'Dev2', 'ctr1', 'Position');
ch2.EncoderType = 'X1';


%%
% Configure quadrature cycle encoding type (X1, X2, or X4). This
% corresponds to the number of counts (counter value increments or
% decrements) output by the encoder for each quadrature cycle (1, 2, or
% 4.), as specified in the encoder datasheet.


%% Read Encoder Position on Demand 
numScans = 10*samplingRate;
start(s,'Continuous')
[encoderData, timestamps] = read(s,numScans,'OutputFormat','Matrix');
% scanData = read(s,numScans);

%[positionData, timestamps] = read(s, daqDuration, 'OutputFormat', 'Matrix');
pause(1)
stop(s)
%%
% This example uses an optical encoder model with a resolution of 2500 
% quadrature cycles per shaft revolution, as specified in the encoder datasheet. 
%
% Convert counter values to angular position (in degrees) using the encoder
% resolution and the encoding type ('X1' in this case).
encoderCPR = 5000;
encoderPosition = encoderData(:,2);
encoderPositionDeg = encoderPosition * 360/encoderCPR;

encoderPosition2 = encoderData(:,3);
encoderPositionDeg2 = encoderPosition2 * 360/encoderCPR;
%%
% By default, counter position readings are unsigned integer values. 
% The counter channels of the DAQ device used in this example are 32-bit, 
% so any counter value read will be in the range 0 to 2^32-1.
% Depending on the application, you may want to obtain signed position values 
% (positive or negative) as decrementing the counter value past zero is a
% discontinuous wraparound to 2^32-1.
% 
% For 32-bit counter channels, use 2^31 as the threshold counter value 
% for conversion to signed position values. The result is valid if the 
% actual position value is in the range -2^31+1 to 2^31.

counterNBits = 32;
signedThreshold = 2^(counterNBits-1);

signedData = encoderData(:,2);
signedData(signedData > signedThreshold) = signedData(signedData > signedThreshold) - 2^counterNBits;

signedData2= encoderData(:,3);
signedData2(signedData2 > signedThreshold) = signedData2(signedData2 > signedThreshold) - 2^counterNBits;

% Calculate encoder position data in degrees.
positionDataDeg = signedData * 360/encoderCPR;
positionDataDeg2 = signedData2 * 360/encoderCPR;

%%
% Plot the signed angular position data acquired for the oscillatory 
% motion of a pendulum.
% figure
% plot(timestamps, positionDataDeg);
% xlabel('Time (s)');
% ylabel('Angular position (deg.)');

% counterNBits = 32;
% signedThreshold = 2^(counterNBits-1);
% encoderCPR = 5000;
% 
% for i = 1:length(optEnc1)
%     if(optEnc1(i) > signedThreshold)
%         optEnc1(i) = optEnc1(i) - 2^counterNBits;
%     end
%     if(optEnc2(i) > signedThreshold)
%         optEnc2(i) = optEnc2(i) - 2^counterNBits;
%     end
% end
% 
% shaftAngleDeg(:,1) = optEnc1 ./ encoderCPR .* 360;
% shaftAngleDeg(:,2) = optEnc2 ./ encoderCPR .* 360;

%% FFT for optical encoder data

Length1=length(positionDataDeg);
rawNFFT1 = 2^nextpow2(Length1); % Next power of 2 from length of y
rawFreq1 = samplingRate/2*linspace(0,1,rawNFFT1/2+1);
rawMag1 = fft( positionDataDeg- mean(positionDataDeg) ,rawNFFT1)/Length1;
PSD = 2*abs(rawMag1(1:rawNFFT1/2+1));

Length2=length(positionDataDeg2);
rawNFFT2 = 2^nextpow2(Length2); % Next power of 2 from length of y
rawFreq2 = samplingRate/2*linspace(0,1,rawNFFT2/2+1);
rawMag2 = fft(positionDataDeg2- mean(positionDataDeg2),rawNFFT2)/Length2;
PSD2 = 2*abs(rawMag2(1:rawNFFT2/2+1));

figure(1)
plot(timestamps, positionDataDeg);
xlabel('Time [s]')
ylabel('Position [deg]')

figure(2)
plot(rawFreq1,PSD,'r');
legend('Raw Position_1')
xlabel('Frequency (Hz)')
ylabel('|Y(f)|')
xlim([0 20])

figure(3)
plot(timestamps, positionDataDeg2);
xlabel('Time [s]')
ylabel('Position [deg]')

figure(4)
plot(rawFreq2,PSD2,'r');
legend('Raw Position_2')
xlabel('Frequency (Hz)')
ylabel('|Y(f)|')
xlim([0 20])

%% Print to file

% text_name = append('Antiphase_','Amplitue_',num2str(A),'_Frequency_',num2str(f),'_IR Frequency_',num2str(IRcycles),'_1','.mat');
% 
% 
% %Specify save path and location
% pathname = fullfile('c:\','Users','inflows','Desktop',text_name); %Set file save location by editting file
% 
% save(pathname,'f','A','IRcycles','timestamps','positionDataDeg','rawFreq1','PSD');
% 
% text_name2 = append('Antiphase_','Amplitue_',num2str(A),'_Frequency_',num2str(f),'_IR Frequency_',num2str(IRcycles),'_2','.mat');
% 
% 
% %Specify save path and location
% pathname2 = fullfile('c:\','Users','inflows','Desktop',text_name2); %Set file save location by editting file
% 
% save(pathname2,'f','A','IRcycles','timestamps','positionDataDeg2','rawFreq2','PSD2');
