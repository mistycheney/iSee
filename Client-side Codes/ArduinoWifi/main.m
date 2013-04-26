HOKUYO_RESPONSE_LEN = 2134;

u=tcpip('192.168.1.2',1000,'NetworkRole','client',...
    'InputBufferSize',HOKUYO_RESPONSE_LEN*10,'OutputBufferSize',5000);
u.Timeout = 10;

tic
fopen(u);
toc

T = 40;
D = zeros(682, T);
ts = zeros(1, T);


tic
for t = 1:T
    t
%     while (u.BytesAvailable < HOKUYO_RESPONSE_LEN)
%     end;
    a = fread(u, HOKUYO_RESPONSE_LEN);
    length(a)
    [D(:,t), ts(t)] = hokuyo_decode(char(a));
end
toc

fclose(u);


%%
P = zeros(682,2,T);

for t = 1:T
    d = D(:,t);
    p = rad2cart_robot_centric(d', 240);
    p = p/1000;
    P(:,:,t) = p;
end

view(P, 0, [-4 4 -4 4]);

figure; title('cumulative time'); xlabel('scan num.'); plot(ts);
% figure; stem(ts(2:end)-ts(1:end-1));
figure; title('time between consecutive scans'); xlabel('time interval length');
hold on; hist(ts(2:end)'-ts(1:end-1)', 300:50:5000); hold off


%%

fprintf(u,'Cheney');

