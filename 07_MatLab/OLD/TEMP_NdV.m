% Standardized Width
width = 5;
t = linspace(0, width, 1000);
% Frequency for exactly 1 period over 5 units
freq = (2 * pi) / width; 
input_signal = square(freq * t); 

% 4th Order Reconstruction (Sum of harmonics 1, 3, 5, 7)
output_signal = (4/pi) * (sin(1*freq*t)/1 + sin(3*freq*t)/3 + ...
                          sin(5*freq*t)/5 + sin(7*freq*t)/7);

figure(); hold on;
%plot(t, input_signal, 'w--', 'LineWidth', 4); % Dashed Input
plot(t, output_signal, 'w', 'LineWidth', 12);  % Bold Solid Output (Harmonics)
ylim([-1.6, 1.6]); 
xlim([-0.1, 5.1]);
axis off; 
hold off;
% Standardized Width
width = 5;
t = linspace(0, width, 1000);
freq = (2 * pi) / width;
% Input: 0 to 1 Square Wave for one period
input_signal = (square(freq * t) + 1) / 2; 

% System Response Calculation (Laplace Domain Step Response)
output_signal = zeros(size(t));
tau = 0.5; % Time constant for the curve shape
dt = t(2) - t(1);
for i = 2:length(t)
    output_signal(i) = output_signal(i-1) + (dt/tau) * (input_signal(i) - output_signal(i-1));
end

figure(); hold on;
%plot(t, input_signal, 'w--', 'LineWidth', 8); % Dashed Input
plot(t, output_signal, 'w', 'LineWidth', 12);  % Bold Solid Output (Exponential rise/fall)
ylim([-0.2, 1.2]);
xlim([-0.1, 5.1]);
axis off; 
hold off;