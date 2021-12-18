%提取bag中消息格式信息
bag = rosbag('/tmp/2021-12-18-13-06-57.bag')

%选择对应的消息
a_dot_topic = select(bag,'Topic','/balance_state_angular');
data_length = 1024;
data_cell = readMessages(a_dot);
data = zeros(data_length,1);
for i=1:data_length
    data(i)=data_cell{i}.Data;
end
data_fil = filter(Hd5,data);
subplot(2,1,1)
plot(data);
hold on;
plot(data_fil);

subplot(2,1,2)
x = fft(data-mean(data));
x_fil = fft(data_fil-mean(data_fil));
plot(abs(x));
% hold on;
% plot(abs(x_fil))

% 采样频率 100 hz 杂波频率 7.3242 hz