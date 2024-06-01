clear all
obs = load('obs.mat','obs').obs;
obs = zeros(4,3,71);
save('no_obs',"obs");
A = obs(:,:,26:46);
B = A;
F = obs(:,:,47:end);
% C = A;
A(find(A==0)) = -11;
F(find(F==6)) = 3;
for k = 1: 21
    for i = 1:4
        B(i,2,k) = B(i,2,k)-12;
        switch A(i,2,k)
            case 2
                A(i,2,k) = A(i,2,k) - 5;
            case 5
                A(i,2,k) = A(i,2,k) - 3;
            case 8
                A(i,2,k) = A(i,2,k) - 3;
            case 10
                A(i,2,k) = A(i,2,k) - 3;
        end
    end
end
for k = 1: 25
    for i = 1:4
        F(i,2,k) = F(i,2,k)+2;
        F(i,3,k) = F(i,3,k)-2;
    end
end
% temp = cat(3, obs(:,:,1:25), B);
% obs = cat(3, temp , obs(:,:,26:end));

% temp4 = cat(3, obs(:,:,1:46), F);
% obs = cat(3, temp4 , obs(:,:,47:end));
obs = cat(3 , A, obs);

% C = B(:,:,15:end);
% D = B(:,:,15:end);
% for k = 1: 7
%     for i = 1:4
%         C(i,2,k) = C(i,2,k)+0.5;
%         D(i,2,k) = C(i,2,k)+3.5;
%     end
% end
% B = cat(3, B, C);
% temp1 = cat(3, obs(:,:,1:25), D);
% temp2 = cat(3, temp1 , obs(:,:,26:end));
% temp3 = cat(3, temp2(:,:,1:53), B);
% obs = cat(3, temp3 , temp2(:,:,54:end));
% obs = cat(3 , A, obs);

save('obs_complex',"obs");