function ud = mirrorVoronoiStrategy(g, uA, vor, xa, ua, xd)
% dird = mirrorVoronoiStrategy(dira, vor)
% Calculates the defender strategy that mirrors the attacker's movement
% relative to the Voronoi line
%
% Mo Chen, Oct. 7, 2013
%

vor_val = eval_u(uA,vor(1,2:end)',vor(2,2:end)',g);
[~, mini] = min(vor_val);
ovproj = vor(:,1+mini)';

dira_perp = ovproj - xa;
dira_perp = dira_perp / norm(dira_perp);
dira_para = [dira_perp(2) -dira_perp(1)];
if dira_para(2)<0, dira_para = -dira_para; end

ua_perp = sum(ua.*dira_perp);
ua_para = sum(ua.*dira_para);

dird_perp = ovproj - xd;
dird_perp = dird_perp / norm(dird_perp);
dird_para = [dird_perp(2) -dird_perp(1)];
if dird_para(2)<0, dird_para = -dird_para; end

ud_perp = abs(ua_perp);
ud_para = ua_para;

ud = ud_perp*dird_perp + ud_para*dird_para;
end
