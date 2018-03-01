function [delta,Eov] = OptimalEov(Fe_1,dpen_1, dpen, kBall,Eov_1,qerror)
Eov = 0;
delta=0;
for delta1=-qerror:qerror/1000:0
    for delta2=-qerror:qerror/1000:0
        Er = Fe_1*((dpen+delta1)-(dpen_1+delta2));
        Ei = kBall*(dpen+delta1)*((dpen+delta1)-(dpen_1+delta2))+Eov_1;
        if Ei-Er > Eov
            Eov = Ei-Er;
            delta = delta1;
        end
    end
end