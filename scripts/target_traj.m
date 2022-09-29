clear all
clc
close all

px = [0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 4.0, 1.0];
py = [0.0, 1.0, 1.0, 4.0, 2.0, 5.0, 2.0, 1.0];

vmax = 0.6; %m/s
t_vec = zeros(size(px,2),1);

for i=2:1:size(px,2)
    if( abs(px(i)-px(i-1)) > abs(py(i)-py(i-1)) )
        t_vec(i) = t_vec(i-1) + abs(px(i)-px(i-1)) / vmax;
    else if ( (px(i)-px(i-1)) < abs(py(i)-py(i-1)) )
            t_vec(i) = t_vec(i-1) + abs(py(i)-py(i-1)) / vmax;
    else if ( abs(px(i)-px(i-1)) == abs(py(i)-py(i-1)) )
            t_vec(i) = t_vec(i-1) + abs(py(i)-py(i-1)) / vmax;
    end
    end
    end
end

[q, dq] = quinticpolytraj([px; py], t_vec,0:0.1:t_vec(end));

csvwrite('../config/target_path.csv', [q', dq'])