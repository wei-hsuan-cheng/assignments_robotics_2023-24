function T = DH2SE3(alpha,a,d,theta)
	d2r = pi / 180;
	r2d = 1 / d2r;
	T = [rotx(alpha * r2d), zeros(3,1); zeros(1,3),1] * [eye(3), [a 0 0].'; zeros(1,3), 1] * [eye(3), [0 0 d].'; zeros(1,3), 1] * [rotz(theta * r2d), zeros(3,1); zeros(1,3), 1];
end