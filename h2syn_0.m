figure(1)
plot(Fh.time,Fh.signals.values,'-k')
hold on;
plot(Fh1.time,Fh1.signals.values,'--r')
axis([0 3 -0.5 2])
xlabel('T/s')
figure(2)
plot(Fe.time,Fe.signals.values,'-k')
hold on;
plot(Fe1.time,Fe1.signals.values,'--r')
axis([0 1 -5 20])
xlabel('T/s')
figure(3)
plot(Vm.time,Vm.signals.values,'k')
hold on;
plot(Vs.time,Vs.signals.values,'--r')
hold on;
plot(Fh1.time,Fh1.signals.values,'.b')
axis([0 10 0 2])
xlabel('T/s')
figure(4)
plot(Fm.time,Fm.signals.values,'-k')
hold on;
plot(Fe1.time,Fe1.signals.values,'--r')
axis([0 10 -4 12])
xlabel('T/s')