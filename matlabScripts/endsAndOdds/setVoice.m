[y, Fs]=audioread('./sounds/20220404_165311.m4a');
% t=find(y(:,1)>0.1);
m=y(:,1);
% m(t)=0;
% sound(m*100,Fs);
%%
figure;
spectrogram(m(Fs*13:Fs*18),320,160,320,Fs,'yaxis');
figure;
plot(y(:,1));
figure;
plot(y(:,2));
% close all

sound(m(Fs*13:Fs*18)*100,Fs);

xx = spectral_subtraction(y(:,1)', 1024, 256, 1024, 0.05, 1)

function output = spectral_subtraction(input, wlen, inc, nlen, a, b)
%------------------------------------
%  input : input signal
%  wlen  : the size of each frame
%  inc   : the size of frame shift
%  nlen  : the size of noisy without speech
%  output: output signal
%------------------------------------
window_function = hamming(wlen);                       % set window function

y = enframe(input, window_function, inc)';             % subframe: y is a matrix with wlen×frame_n
frames_n = size(y, 2);                                 % the number of frames

y_fft = fft(y);                                        % fft
y_fft_amplitude = abs(y_fft);                          % amplitude
y_fft_energy = y_fft_amplitude.^2;                     % energy
y_fft_phase = angle(y_fft);                            % phase

noisy_energy = mean(y_fft_energy(:,1:nlen), 2);        % noisy energy is a vector with wlen×1

output_energy = ones(wlen, frames_n);
for ii = 1:frames_n
    for jj = 1:wlen
        if y_fft_energy(jj, ii) > a*noisy_energy(jj)
            output_energy(jj, ii) = y_fft_energy(jj, ii) - a*noisy_energy(jj);
        else
            output_energy(jj, ii) = b*y_fft_energy(jj, ii);
        end        
    end
end
output_amplitude = sqrt(output_energy);                % output amplitude

output = OverlapAdd2(output_amplitude, y_fft_phase, wlen, inc);

end