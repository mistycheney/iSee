function [d, timestamp] = hokuyo_decode(str)

A = textscan(str, '%s', 'delimiter', sprintf('\n'));
status = A{1}{2};

assert(strcmp(status,'00P'));

% if (~strcmp(status,'00P'))
%     display(status);
%     d = 0; timestamp = 0;
%     return
% end
    
timestamp_str = A{1}{3}(1:4);
timestamp = ts_decode(timestamp_str, 4);

Astrip = cellfun(@(x) x(1:end-1), A{1}(4:end-1), 'UniformOutput', 0);
Astring = [Astrip{:}];
numScan = length(Astring)/3;

% tic;
% Atriplet = mat2cell(Astring, 1, 3*ones(1, numScan));
% P = cellfun(@(x) ts_decode(x, 3), Atriplet);
% toc;

d = zeros(numScan,1);

% tic;
for i=1:numScan
    d(i) = ts_decode(Astring(3*(i-1)+1:3*(i-1)+3), 3);
end
% toc;

% P

