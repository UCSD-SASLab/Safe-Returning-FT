function [xv_aug,yv_aug] = AugObsVx(xv,yv,TEB)

xv_aug = xv;
xv_aug = xv_aug - TEB(1);
xv_aug(2:3) = xv_aug(2:3) + 2*TEB(1);
yv_aug = yv;
yv_aug = yv_aug - TEB(2);
yv_aug(3:4) = yv_aug (3:4) + 2*TEB(2);


